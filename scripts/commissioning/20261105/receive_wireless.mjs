import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
import readline from 'node:readline';

// Receive actual Pi camera/scan/status samples through an authenticated SSH link.
// This observer provides no actuator controls. It does not measure one-way latency.
const base = path.resolve(process.argv[2]);
const runId = process.argv[3];
const duration = Number(process.argv[4] || 120);
if (!/^run-[0-9TZ-]+$/.test(runId) || !Number.isFinite(duration) || duration < 30 || duration > 300) {
  throw new Error('Invalid run ID or duration');
}
const out = path.join(base, runId);
fs.mkdirSync(out, { recursive: false });
fs.mkdirSync(path.join(out, 'frames'));
const remoteOut = `/home/group5/dase7503-commissioning/wireless-20261105/${runId}`;
const command = `set -e\nsource /home/group5/dase7503-commissioning/session.env\nexec python3 -u /home/group5/dase7503-commissioning/wireless-20261105/wireless_static_soak.py --output '${remoteOut}' --duration ${duration}\n`;
const encoded = Buffer.from(command).toString('base64');
const args = [
  '-i', path.join(process.env.USERPROFILE, '.ssh', 'dase7503_group5_ed25519'),
  '-o', 'IdentitiesOnly=yes', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
  '-o', 'ServerAliveInterval=10', '-o', 'ServerAliveCountMax=2',
  '-o', 'StrictHostKeyChecking=yes',
  '-o', `UserKnownHostsFile=${path.join(process.env.LOCALAPPDATA, 'Codex', 'pi-debug', '20261105', 'pi_known_hosts')}`,
  '-o', 'GlobalKnownHostsFile=NUL', '-o', 'HostKeyAlias=192.168.137.194',
  'group5@192.168.0.207', `printf %s '${encoded}' | base64 -d | bash`,
];
const startedAt = new Date().toISOString();
const startedMono = performance.now();
const logFd = fs.openSync(path.join(out, 'received-telemetry.ndjson'), 'wx');
const errFd = fs.openSync(path.join(out, 'ssh-stderr.log'), 'wx');
const issues = [];
let report = null, previousSeq = null, previousReceive = null, lastProgress = 0;
let receivedBytes = 0, jpegCount = 0, scanCount = 0, malformedLines = 0;
const measuringTimes = [], measuringPreviewStamps = [], measuringScanStamps = [];
let maximumGapMs = 0;
const child = spawn('C:\\Windows\\System32\\OpenSSH\\ssh.exe', args, { windowsHide: true, stdio: ['ignore', 'pipe', 'pipe'] });
child.stderr.on('data', b => fs.writeSync(errFd, b));
child.stdout.on('data', b => { receivedBytes += b.length; });
child.on('error', e => issues.push(`ssh: ${e.message}`));
const timeout = setTimeout(() => {
  issues.push('SSH stream exceeded bounded execution window; remote cleanup must be checked');
  child.kill();
}, (duration + 100) * 1000);
const lines = readline.createInterface({ input: child.stdout });
lines.on('line', line => {
  const receivedAt = new Date().toISOString(), mono = performance.now();
  try {
    const row = JSON.parse(line);
    if (row.type === 'report') report = row;
    if (row.type === 'telemetry') {
      if (previousSeq !== null && row.seq !== previousSeq + 1) issues.push(`sequence discontinuity ${previousSeq} -> ${row.seq}`);
      previousSeq = row.seq;
      if (row.phase === 'measuring') {
        measuringTimes.push(mono);
        if (previousReceive !== null) maximumGapMs = Math.max(maximumGapMs, mono - previousReceive);
        previousReceive = mono;
      }
      if (row.preview?.jpeg_base64) {
        const bytes = Buffer.from(row.preview.jpeg_base64, 'base64');
        if (bytes.length < 4 || bytes[0] !== 255 || bytes[1] !== 216 || bytes.at(-2) !== 255 || bytes.at(-1) !== 217) {
          issues.push(`invalid JPEG at seq ${row.seq}`);
        } else {
          jpegCount++;
          fs.writeFileSync(path.join(out, 'latest-preview.jpg'), bytes);
          if (jpegCount === 1 || jpegCount % 15 === 0) fs.writeFileSync(path.join(out, 'frames', `${String(row.seq).padStart(4, '0')}.jpg`), bytes);
          if (row.phase === 'measuring') measuringPreviewStamps.push(row.preview.source_stamp_s);
        }
        delete row.preview.jpeg_base64;
        row.preview.received_bytes = bytes.length;
        row.preview.sha256 = createHash('sha256').update(bytes).digest('hex');
      }
      if (row.scan?.points?.length) {
        scanCount++;
        fs.writeFileSync(path.join(out, 'latest-scan.json'), JSON.stringify(row.scan, null, 2));
        if (row.phase === 'measuring') measuringScanStamps.push(row.scan.stamp_s);
      }
      if (mono - lastProgress > 15000) {
        console.log(JSON.stringify({ phase: row.phase, seq: row.seq, elapsed_s: row.elapsed_s,
          received_previews: jpegCount, received_scans: scanCount, received_bytes: receivedBytes }));
        lastProgress = mono;
      }
    }
    fs.writeSync(logFd, JSON.stringify({ desktop_received_at: receivedAt,
      desktop_elapsed_s: (mono - startedMono) / 1000, ...row }) + '\n');
  } catch (e) {
    malformedLines++;
    fs.writeSync(errFd, `UNPARSED_STDOUT ${line.slice(0, 2000)}\n`);
  }
});
const closed = await new Promise(resolve => child.on('close', (code, signal) => resolve({ code, signal })));
clearTimeout(timeout);
fs.closeSync(logFd); fs.closeSync(errFd);
const increasing = xs => xs.length > 1 && xs.every((x, i) => Number.isFinite(x) && (i === 0 || x > xs[i - 1]));
const enough = measuringTimes.length >= duration * 0.8;
const receiver = {
  started_at: startedAt, ended_at: new Date().toISOString(), run_id: runId,
  target: 'group5@192.168.0.207', verified_known_host_alias: '192.168.137.194',
  data_path: 'Pi local ROS -> sampled JPEG + scan/status JSON -> SSH over wlan0 -> Windows receiver',
  full_rate_ros_dds_over_wifi_tested: false, one_way_network_latency_measured: false,
  grounded_motion_tested: false, wifi_loss_stopping_tested: false,
  expected_measurement_duration_s: duration, ssh_exit: closed,
  received_bytes: receivedBytes, valid_jpeg_previews: jpegCount, scan_snapshots: scanCount,
  measuring_snapshots: measuringTimes.length, maximum_measuring_receive_gap_s: maximumGapMs / 1000,
  preview_source_stamps_increasing: increasing(measuringPreviewStamps),
  scan_source_stamps_increasing: increasing(measuringScanStamps),
  malformed_lines: malformedLines, issues,
  remote_static_passed: report?.passed === true,
  passed: closed.code === 0 && report?.passed === true && issues.length === 0 && malformedLines === 0 && enough
    && maximumGapMs < 3000 && increasing(measuringPreviewStamps) && increasing(measuringScanStamps),
  remote_report: report,
};
fs.writeFileSync(path.join(out, 'receiver-report.json'), JSON.stringify(receiver, null, 2));
console.log(JSON.stringify({ type: 'receiver_report', passed: receiver.passed, run_id: runId,
  received_bytes: receivedBytes, previews: jpegCount, scans: scanCount, snapshots: measuringTimes.length,
  max_receive_gap_s: maximumGapMs / 1000, ssh_exit: closed, issues,
  remote_failures: report?.failures ?? ['No final remote report'], report_path: path.join(out, 'receiver-report.json') }));
process.exitCode = receiver.passed ? 0 : 1;
