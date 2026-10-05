#!/usr/bin/env python3
"""Install a built e-ink lab into the existing sp2 static site, or stage it."""
import argparse
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent

def install(site):
    public = HERE / 'public'
    required = ['emulator.wasm', *[f'{app}-{device}.wasm' for app in ['tinta', 'lila'] for device in ['x3', 'x4classic', 'x4pro']]]
    for name in required:
        if not (public/'runtime'/name).is_file(): raise RuntimeError(f'Build runtime/{name} before integration')
    for name in ['tinta-screen.png', 'lila-screen.png']:
        if not (public/'content'/name).is_file(): raise RuntimeError(f'Capture content/{name} using the browser test first')
    plans = json.loads((public/'releases/index.json').read_text())
    if not plans: raise RuntimeError('Package device images before integration')
    shutil.copytree(public, site/'e-ink', dirs_exist_ok=True, ignore=shutil.ignore_patterns('compat'))
    page = (site/'index.html').read_text()
    if 'id="e-ink"' not in page:
        marker = '      <div class="journey">'
        if marker not in page: raise RuntimeError('Homepage changed: review the feature insertion point')
        page = page.replace(marker, (HERE/'home-section.html').read_text()+marker, 1)
    if 'href="#e-ink"' not in page:
        page = page.replace('<a href="#work">Work</a>', '<a href="#e-ink">E-ink R&amp;D</a>\n        <a href="#work">Work</a>', 1)
    if '/assets/eink.css' not in page:
        page = page.replace('<link rel="stylesheet" href="/assets/site.css">', '<link rel="stylesheet" href="/assets/site.css">\n  <link rel="stylesheet" href="/assets/eink.css">', 1)
    (site/'index.html').write_text(page)
    shutil.copyfile(HERE/'home-section.css', site/'assets/eink.css')
    headers = (site/'_headers').read_text()
    # Cloudflare combines duplicate headers across matching rules. Extend the
    # existing CSP instead of accidentally appending a second restrictive CSP.
    headers = headers.replace("script-src 'self';", "script-src 'self' 'wasm-unsafe-eval'; worker-src 'self' blob:; connect-src 'self';")
    headers = headers.replace('payment=(), usb=(),', 'payment=(), usb=(), serial=(self),')
    if '/e-ink/*' not in headers:
        headers += '\n/e-ink/*\n  Cross-Origin-Embedder-Policy: require-corp\n  Cache-Control: no-cache\n\n/e-ink/*.mjs\n  Content-Type: text/javascript\n\n/e-ink/*.wasm\n  Content-Type: application/wasm\n'
    (site/'_headers').write_text(headers)
    sitemap = (site/'sitemap.xml').read_text()
    if 'https://subtlepath.com/e-ink/' not in sitemap:
        sitemap = sitemap.replace('</urlset>', '  <url><loc>https://subtlepath.com/e-ink/</loc></url>\n</urlset>')
    (site/'sitemap.xml').write_text(sitemap)
    readme = site.parent/'README.md'
    if readme.is_file():
        text = readme.read_text()
        if '## E-ink product lab' not in text:
            text += '''

## E-ink product lab

`site/e-ink/` contains real Tinta and lila firmware compiled to WebAssembly,
three hardware profiles, and a Web Serial installer with exact-board images.
The homepage links to the lab in its prominent product R&D section.

Build recipes and tests live in the sibling FreeInk SDK at
`tools/simulator/web/README.md`. Rebuild and run its `integrate.py --site ../sp2/site`
to refresh the static output. Generated WASM and firmware images are included
here so the existing static deployment remains sufficient.

Preview with isolation headers rather than a plain HTTP server:

```sh
cd ../freeink-sdk
python3 tools/simulator/web/serve.py --directory ../sp2/site --port 8767
```

The lab needs the supplied `_headers` for threaded WASM and same-origin Web
Serial. The installer requires desktop Chrome/Edge and HTTPS (localhost works
for development). Images are development builds; no real USB write was tested
during implementation. Integration does not publish the site.
'''
            readme.write_text(text)
    print(f'Installed e-ink lab, {len(plans)} firmware plans, homepage feature and host headers into {site}')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--site', type=Path, default=HERE.parents[2].parent/'sp2/site')
    parser.add_argument('--stage', type=Path, help='Copy the existing site here and integrate the lab for review')
    args = parser.parse_args()
    site = args.site.resolve()
    if args.stage:
        staged = args.stage.resolve()
        shutil.copytree(site, staged, dirs_exist_ok=True)
        site = staged
    install(site)

if __name__ == '__main__': main()
