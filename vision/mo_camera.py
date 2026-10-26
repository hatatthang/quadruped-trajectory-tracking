"""Double-click launcher. Reuse the local service; never auto-open a camera."""
from datetime import datetime
import http.client
import json
from pathlib import Path
import socket
import subprocess
import sys
import time
import webbrowser

ROOT = Path(__file__).resolve().parent
PORT = 8765
URL = f'http://127.0.0.1:{PORT}/final'


def service_ready():
    connection = http.client.HTTPConnection('127.0.0.1', PORT, timeout=1)
    try:
        connection.request('GET', '/api/state')
        response = connection.getresponse()
        if response.status != 200:
            return False
        state = json.loads(response.read(1024 * 1024))
        connection.request('GET', '/api/final/state')
        final_response = connection.getresponse()
        final_response.read()
        if final_response.status != 200:
            return False
        session = state.get('session', {}).get('id')
        return (isinstance(session, str) and Path(session).name == session
                and (ROOT/'data'/session/'session.json').is_file()
                and state.get('camera', {}).get('name') == 'Rapoo Camera')
    except (OSError, ValueError, AttributeError, http.client.HTTPException):
        return False
    finally:
        connection.close()


def port_in_use():
    try:
        with socket.create_connection(('127.0.0.1', PORT), timeout=.5):
            return True
    except OSError:
        return False


def select_session(root=ROOT):
    """Pick the latest saved Rapoo calibration, excluding incomplete sessions."""
    data_dir = root/'data'
    for folder in sorted(data_dir.iterdir(), key=lambda p: p.name, reverse=True) if data_dir.is_dir() else []:
        if not folder.is_dir() or folder.resolve().parent != data_dir.resolve():
            continue
        try:
            document = json.loads((folder/'session.json').read_text(encoding='utf-8-sig'))
            calibration = document.get('calibration', {})
            if (document.get('session_id') == folder.name
                    and document.get('camera') == 'Rapoo Camera'
                    and document.get('pattern') == [9, 6]
                    and document.get('samples')
                    and calibration.get('status') in ('candidate', 'review_required')
                    and calibration.get('camera_matrix')
                    and calibration.get('distortion_coefficients')):
                return folder.name
        except (OSError, ValueError, AttributeError):
            continue
    raise RuntimeError('Khong tim thay phien hieu chuan Rapoo da luu. Khong tao de len du lieu cu.')


def main():
    try:
        if not service_ready():
            if port_in_use():
                raise RuntimeError('Cong 8765 dang duoc dung nhung trang camera chua san sang. Khong mo them chuong trinh; hay kiem tra phien dang chay.')
            session = select_session()
            print('Dang mo Camera Ver0, vui long cho...')
            log_path = ROOT/'launcher_server.log'
            with log_path.open('a', encoding='utf-8') as log:
                log.write('\nLauncher ' + datetime.now().isoformat() + ' - resume ' + session + '\n')
                log.flush()
                process = subprocess.Popen(
                    [sys.executable, '-B', str(ROOT/'camera_ver0.py'), '--no-camera',
                     '--port', str(PORT), '--resume-session', session],
                    cwd=str(ROOT), stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                    creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            deadline = time.monotonic() + 30
            while not service_ready():
                if process.poll() is not None:
                    raise RuntimeError('Chuong trinh chua mo duoc. Xem file: ' + str(log_path))
                if time.monotonic() >= deadline:
                    raise RuntimeError('Chuong trinh van dang khoi dong. Thu mo ' + URL + ' sau it giay; xem launcher_server.log neu van loi.')
                time.sleep(.4)
        if not webbrowser.open(URL, new=2):
            print('Mo trinh duyet va truy cap: ' + URL)
            input('Nhan Enter de dong cua so nay...')
        return 0
    except (OSError, RuntimeError) as exc:
        print('Khong mo duoc trang camera: ' + str(exc))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
