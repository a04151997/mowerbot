"""網頁人機介面裡的 Lichtblick (2026-10-22 will：從網頁一鍵打開、自動連線，不用自己開程式、打網址)。

Lichtblick 網頁版 (官方 ghcr.io/lichtblick-suite/lichtblick 映像檔裡的靜態檔) 由安裝檔放在
/usr/local/share/mowerbot-lichtblick，web.py 在 /lichtblick/ 底下送出去，跟網頁同一個 port，
所以熱點上沒有網路也能用。網頁的按鈕開
    /lichtblick/?ds=foxglove-websocket&ds.url=ws://<車子>:8765
Lichtblick 讀到 ds 參數就直接連 car-start 開的 foxglove_bridge (只能看，見 foxglove.yaml)。

預設版面 (地圖、光達、割草路線、禁區、外框) 在 share/mowerbot_hmi/web/lichtblick_layout.json，
送 index.html 時塞進官方留的 LICHTBLICK_SUITE_DEFAULT_LAYOUT_PLACEHOLDER (官方 Docker 也是這樣塞)。
Lichtblick 只有在瀏覽器還沒有版面時才用預設版面，之後用的人自己改的版面會留著。

這裡不用 ROS，可以單獨測。
"""
import json
import os

ROOT = os.environ.get('MOWER_LICHTBLICK_DIR', '/usr/local/share/mowerbot-lichtblick')
PREFIX = '/lichtblick/'
PLACEHOLDER = '/*LICHTBLICK_SUITE_DEFAULT_LAYOUT_PLACEHOLDER*/'
# 檔名有雜湊的 (main.2f02….js) 內容不會變，瀏覽器存一週；index.html 每次都問
CACHE_LONG = 'public, max-age=604800'
TYPES = {
    '.html': 'text/html; charset=utf-8', '.txt': 'text/plain; charset=utf-8', '.css': 'text/css',
    '.js': 'application/javascript', '.mjs': 'application/javascript',
    '.json': 'application/json', '.map': 'application/json', '.wasm': 'application/wasm',
    '.png': 'image/png', '.svg': 'image/svg+xml', '.ico': 'image/x-icon',
    '.jpg': 'image/jpeg', '.gif': 'image/gif', '.webp': 'image/webp',
    '.ttf': 'font/ttf', '.woff': 'font/woff', '.woff2': 'font/woff2',
}


class Viewer:
    """/lichtblick/ 底下的檔案。root = 安裝檔解開的 Lichtblick 網頁版，layout = 預設版面 JSON 的路徑。"""

    def __init__(self, root=ROOT, layout=None):
        self.root = os.path.realpath(root)
        self.layout = layout
        self._index = None          # (mtime 組合, bytes)

    def installed(self):
        return os.path.isfile(os.path.join(self.root, 'index.html'))

    def version(self):
        """安裝檔寫的 VERSION (例如 1.29.1)，沒有就空字串。"""
        try:
            with open(os.path.join(self.root, 'VERSION'), encoding='utf-8') as fh:
                return fh.read().strip()[:20]
        except OSError:
            return ''

    def index_html(self):
        """index.html，預設版面塞進去 (檔案改了才重讀)。"""
        path = os.path.join(self.root, 'index.html')
        has_layout = bool(self.layout) and os.path.isfile(self.layout)
        key = (os.path.getmtime(path), os.path.getmtime(self.layout) if has_layout else 0)
        if self._index and self._index[0] == key:
            return self._index[1]
        with open(path, encoding='utf-8') as fh:
            html = fh.read()
        if has_layout and PLACEHOLDER in html:
            with open(self.layout, encoding='utf-8') as fh:
                layout = json.load(fh)
            # 放在 <script> 裡：</ 要拆開，不然版面裡的字串會提早結束 script
            text = json.dumps(layout, ensure_ascii=False).replace('</', '<\\/')
            html = html.replace(PLACEHOLDER, text)
        body = html.encode('utf-8')
        self._index = (key, body)
        return body

    def resolve(self, url_path, accept_gzip):
        """網址 -> (檔案路徑, Content-Type, 額外標頭) 或 None (沒有這個檔)。

        index.html 另外處理 (要塞版面)，這裡回傳 ('index', …)。有先壓好的 .gz 而且瀏覽器收 gzip 就送 .gz。
        """
        rel = url_path[len(PREFIX):] if url_path.startswith(PREFIX) else None
        if rel is None:
            return None
        if rel in ('', 'index.html'):
            return ('index', TYPES['.html'], {'Cache-Control': 'no-cache'})
        path = os.path.realpath(os.path.join(self.root, rel))
        # 不能跑出 Lichtblick 的資料夾 (../../etc/passwd 這種)
        if not path.startswith(self.root + os.sep) or not os.path.isfile(path):
            return None
        ctype = TYPES.get(os.path.splitext(path)[1].lower(), 'application/octet-stream')
        extra = {'Cache-Control': CACHE_LONG, 'Vary': 'Accept-Encoding'}
        if accept_gzip and os.path.isfile(path + '.gz'):
            extra['Content-Encoding'] = 'gzip'
            path += '.gz'
        return (path, ctype, extra)


def accepts_gzip(header):
    """Accept-Encoding 有沒有 gzip (q=0 的不算)。"""
    for part in (header or '').split(','):
        name, _, params = part.strip().partition(';')
        if name.strip().lower() == 'gzip':
            return params.replace(' ', '') not in ('q=0', 'q=0.0', 'q=0.00', 'q=0.000')
    return False
