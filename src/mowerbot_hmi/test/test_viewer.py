"""viewer.py：網頁裡的 Lichtblick (2026-10-22)。"""
import json

from mowerbot_hmi import viewer


def make_root(tmp_path, placeholder=True):
    root = tmp_path / 'lb'
    root.mkdir()
    marker = viewer.PLACEHOLDER if placeholder else ''
    html = '<script src="main.abc.js"></script><script>D = [%s][0];</script>' % marker
    (root / 'index.html').write_text(html, encoding='utf-8')
    (root / 'main.abc.js').write_text('console.log(1)', encoding='utf-8')
    (root / 'main.abc.js.gz').write_bytes(b'\x1f\x8b')
    (root / 'x.wasm').write_bytes(b'\0asm')
    (root / 'VERSION').write_text('1.29.1\n', encoding='utf-8')
    (tmp_path / 'secret.txt').write_text('no', encoding='utf-8')
    layout = tmp_path / 'layout.json'
    layout.write_text(json.dumps({'layout': '3D!a', 'note': '</script>'}), encoding='utf-8')
    return root, layout


def test_not_installed_without_index(tmp_path):
    v = viewer.Viewer(str(tmp_path / 'nothing'))
    assert not v.installed() and v.version() == ''


def test_resolve_types_gzip_and_cache(tmp_path):
    root, layout = make_root(tmp_path)
    v = viewer.Viewer(str(root), str(layout))
    assert v.installed() and v.version() == '1.29.1'
    assert v.resolve('/lichtblick/', True)[0] == 'index'
    assert v.resolve('/lichtblick/index.html', True)[0] == 'index'
    path, ctype, extra = v.resolve('/lichtblick/main.abc.js', True)
    assert path.endswith('main.abc.js.gz') and ctype == 'application/javascript'
    assert extra['Content-Encoding'] == 'gzip' and 'max-age' in extra['Cache-Control']
    path, _, extra = v.resolve('/lichtblick/main.abc.js', False)
    assert path.endswith('main.abc.js') and 'Content-Encoding' not in extra
    assert v.resolve('/lichtblick/x.wasm', True)[1] == 'application/wasm'   # 不對的話瀏覽器不肯編譯


def test_resolve_refuses_outside_and_missing(tmp_path):
    root, layout = make_root(tmp_path)
    v = viewer.Viewer(str(root), str(layout))
    assert v.resolve('/lichtblick/../secret.txt', False) is None
    assert v.resolve('/lichtblick/%2e%2e/secret.txt', False) is None
    assert v.resolve('/lichtblick/nope.js', False) is None
    assert v.resolve('/other/main.abc.js', False) is None


def test_index_gets_layout_and_script_end_is_escaped(tmp_path):
    root, layout = make_root(tmp_path)
    html = viewer.Viewer(str(root), str(layout)).index_html().decode('utf-8')
    assert viewer.PLACEHOLDER not in html and '"layout": "3D!a"' in html
    assert html.count('</script>') == 2                      # 版面裡的 </script> 沒有提早結束 script
    root2 = tmp_path / 'b'
    root2.mkdir()
    (root2 / 'index.html').write_text('<p>old</p>', encoding='utf-8')
    assert viewer.Viewer(str(root2), str(layout)).index_html() == b'<p>old</p>'   # 沒有位置就照原樣


def test_accepts_gzip():
    assert viewer.accepts_gzip('gzip, deflate, br')
    assert viewer.accepts_gzip('br;q=1.0, gzip;q=0.8')
    assert not viewer.accepts_gzip('gzip;q=0, br')
    assert not viewer.accepts_gzip('')
    assert not viewer.accepts_gzip(None)
