"""Save the exact confirming image and its metadata; failures are nonfatal."""
from datetime import datetime
import json
from pathlib import Path
import uuid
import cv2


def save_confirmation(directory, payload, detector, frame, stamp_ns, source):
    path = Path(directory).expanduser()
    path.mkdir(parents=True, exist_ok=True)
    key = datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '-' + uuid.uuid4().hex[:8]
    image = path / (key + '.png')
    if not cv2.imwrite(str(image), frame):
        raise OSError(f'Could not write confirmation image: {image}')
    record = {'payload': payload, 'detector': detector, 'source': source,
              'received_at': datetime.now().astimezone().isoformat(),
              'image_stamp_ns': int(stamp_ns), 'image': image.name,
              'kind': 'exact_confirming_input_frame', 'physical_acceptance': False}
    (path / (key + '.json')).write_text(json.dumps(record, indent=2, ensure_ascii=False), encoding='utf-8')
    return record
