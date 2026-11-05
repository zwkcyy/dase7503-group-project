param([Parameter(Mandatory=$true)][string]$OutFile, [int]$Count=150)
$ErrorActionPreference = 'Stop'
if (Test-Path -LiteralPath $OutFile) { throw 'Refusing to overwrite existing network evidence' }
$probe = [System.Net.NetworkInformation.Ping]::new()
$watch = [Diagnostics.Stopwatch]::StartNew()
$rows = [Collections.Generic.List[object]]::new()
try {
    for ($index=0; $index -lt $Count; $index++) {
        $tick = $watch.ElapsedMilliseconds
        try {
            $reply = $probe.Send('192.168.0.207', 900)
            $rows.Add([pscustomobject]@{time=(Get-Date).ToString('o'); elapsed_s=$watch.Elapsed.TotalSeconds; status=$reply.Status.ToString(); rtt_ms=$reply.RoundtripTime})
        } catch {
            $rows.Add([pscustomobject]@{time=(Get-Date).ToString('o'); elapsed_s=$watch.Elapsed.TotalSeconds; status='Exception'; error=$_.Exception.Message; rtt_ms=$null})
        }
        $wait = 1000 - ($watch.ElapsedMilliseconds - $tick)
        if ($index -lt $Count-1 -and $wait -gt 0) { Start-Sleep -Milliseconds $wait }
    }
} finally {
    $probe.Dispose()
    $good = @($rows | Where-Object status -eq 'Success')
    $times = @($good | ForEach-Object rtt_ms | Sort-Object)
    $stats = $good | Measure-Object -Property rtt_ms -Average -Minimum -Maximum
    $report = [pscustomobject]@{
        target='192.168.0.207'; test='ICMP while real sensor previews and status stream over wireless SSH';
        duration_s=$watch.Elapsed.TotalSeconds; sent=$rows.Count; received=$good.Count;
        loss_percent= $(if ($rows.Count) {100.0*($rows.Count-$good.Count)/$rows.Count} else {$null});
        rtt_min_ms=$stats.Minimum; rtt_mean_ms=$stats.Average; rtt_max_ms=$stats.Maximum;
        rtt_p95_ms= $(if($times.Count) {$times[[Math]::Max(0,[Math]::Ceiling(.95*$times.Count)-1)]} else {$null});
        samples=$rows
    }
    [IO.File]::WriteAllText($OutFile,($report | ConvertTo-Json -Depth 6),[Text.UTF8Encoding]::new($false))
    $report | Select-Object target,duration_s,sent,received,loss_percent,rtt_min_ms,rtt_mean_ms,rtt_max_ms,rtt_p95_ms | ConvertTo-Json
}
