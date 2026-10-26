"""Mo lai luong thu hinh cua Camera Ver0 ma KHONG tat chuong trinh.

Dung khi trang web bao camera 'live' nhung hinh dung, den, hoac fps tut
xuong 1. Nguyen nhan la handle DirectShow cu bi ket hoac driver thuong
luong nham toc do khung hinh. Tat roi mo lai luong thu hinh la du.

Script nay KHONG tat server nen phien hieu chuan va mat phang van con
trong bo nho. Nhung epoch camera doi, nen sau khi chay xong phai vao
trang /line bam 'Khoi phuc mat phang vua dung' roi 'Bat xem vi tri'.
"""
import json
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def load_connection():
    document = json.loads((ROOT/'connection.json').read_text(encoding='utf-8-sig'))
    return document['url'].rstrip('/'), document['token']


def get_state(url):
    with urllib.request.urlopen(url+'/api/state', timeout=8) as response:
        return json.loads(response.read())


def show(url, label):
    try:
        camera = get_state(url).get('camera', {})
        fps = camera.get('fps')
        age = camera.get('age_seconds')
        print('  %-10s %-8s %s fps, anh moi %s giay'
              % (label, camera.get('status'),
                 ('%.1f' % fps) if isinstance(fps, (int, float)) else '?',
                 ('%.2f' % age) if isinstance(age, (int, float)) else '?'))
    except (OSError, ValueError) as exc:
        print('  %-10s khong doc duoc trang thai: %s' % (label, exc))


def post(url, token, route, payload=None):
    request = urllib.request.Request(
        url+route, data=json.dumps(payload or {}).encode('utf-8'),
        headers={'Content-Type': 'application/json', 'X-Session-Token': token},
        method='POST')
    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            json.loads(response.read())
        return True
    except urllib.error.HTTPError as exc:
        try:
            print('     (%s tra ve: %s)' % (route, json.loads(exc.read()).get('message', '')))
        except (OSError, ValueError):
            pass
        return False
    except OSError as exc:
        print('     (%s loi: %s)' % (route, exc))
        return False


def main():
    try:
        url, token = load_connection()
    except (OSError, ValueError, KeyError):
        print('Khong doc duoc connection.json. Chuong trinh camera chua chay?')
        print('Hay bam MO_CAMERA.cmd truoc.')
        return

    try:
        get_state(url)
    except OSError:
        print('Chuong trinh camera khong phan hoi tai '+url)
        print('Hay bam MO_CAMERA.cmd truoc.')
        return

    print('Trang thai truoc khi mo lai:')
    show(url, 'truoc')

    print('Dang dung theo doi line va tat luong thu hinh...')
    post(url, token, '/api/line/cancel')
    post(url, token, '/api/stop')
    time.sleep(3)

    print('Dang mo lai luong thu hinh...')
    post(url, token, '/api/start')

    for _ in range(5):
        time.sleep(2.5)
        show(url, 'sau')

    camera = get_state(url).get('camera', {})
    fps = camera.get('fps') or 0
    print()
    if camera.get('status') == 'live' and fps >= 3:
        print('XONG. Camera da chay lai binh thuong.')
    else:
        print('CHUA DAT. Camera van bat thuong.')
        print('Hay rut cap USB cua camera, cam lai vao cong khac, roi chay lai script nay.')
    print()
    print('Buoc tiep theo tren trang /line:')
    print('  1. Tai lai trang')
    print('  2. Tick o "Camera/tripod, lay net va do phan giai giu nguyen"')
    print('  3. Bam "Khoi phuc mat phang vua dung"')
    print('  4. Bam "Bat xem vi tri"')
    print('Tick o do la dung su that: camera khong bi cham vao, chi co ket noi')
    print('phan mem mo lai. Chi KHONG duoc tick neu ban that su da di chuyen camera.')


if __name__ == '__main__':
    main()
