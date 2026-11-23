// Draws frames FROM..TO (exclusive) of a page to OUT/NNNNN.png.
//   node render.mjs PAGE.html OUT [FROM TO]
// The page must define window.renderFrame(k) and window.totalFrames.
import { createRequire } from 'module';
import { mkdirSync } from 'fs';
import path from 'path';
import { pathToFileURL } from 'url';
const { chromium } = createRequire(import.meta.url.replace(/[^/]*$/, '') + 'x.js')('playwright');

const [page_, out, from_, to_] = process.argv.slice(2);
mkdirSync(out, { recursive: true });
const browser = await chromium.launch({
  args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader',
         '--ignore-gpu-blocklist', '--allow-file-access-from-files', '--force-color-profile=srgb'],
});
const page = await browser.newPage({ viewport: { width: 1920, height: 1080 }, deviceScaleFactor: 1 });
page.on('console', (m) => { if (m.type() === 'error') console.error('page:', m.text()); });
page.on('pageerror', (e) => console.error('page error:', e.message));
await page.goto(pathToFileURL(path.resolve(page_)).href);
await page.evaluate(() => document.fonts.ready);
await page.waitForFunction(() => window.totalFrames > 0, null, { timeout: 120000 });
const total = await page.evaluate(() => window.totalFrames);
const from = from_ ? Number(from_) : 0, to = Math.min(total, to_ ? Number(to_) : total);
for (let k = from; k < to; k++) {
  await page.evaluate((k) => window.renderFrame(k), k);
  await page.screenshot({ path: path.join(out, String(k).padStart(5, '0') + '.png') });
  if ((k - from) % 100 === 0) console.log(`frame ${k} / ${to}`);
}
await browser.close();
