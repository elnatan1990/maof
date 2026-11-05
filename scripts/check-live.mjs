// Ask the DEPLOYED companion page the questions that matter. Read-only; exits 1 on any failure.
//   node scripts/check-live.mjs --base=https://<host>
const base = (process.argv.find(a => a.startsWith("--base=")) || "--base=https://maof-mep.vercel.app").slice(7).replace(/\/$/, "");
const failures = [];
const check = (ok, what) => { console.log(`${ok ? "ok " : "FAIL"} ${what}`); if (!ok) failures.push(what); };

const home = await fetch(base + "/", { redirect: "manual" });
const html = await home.text();
check(home.status === 200, `GET / -> ${home.status}`);
check((home.headers.get("content-type") || "").includes("text/html"), "home is text/html");
check(/noindex/.test(home.headers.get("x-robots-tag") || ""), "X-Robots-Tag noindex (a take-home page is not for search engines)");
check(/<title>MEP Coordination Spike<\/title>/.test(html), "title present");
check(/Elnatan Davidovitch/.test(html), "author line present");
check(/three@0\.160\.0\/build\/three\.module\.js/.test(html), "three.js pinned to 0.160.0");
check(!/googletagmanager|clarity\.ms|facebook\.net|connect\.facebook|ads\.openai|googleads/.test(html), "no tracking scripts");

const data = await fetch(base + "/scenario.json");
let scenarios = {};
try { scenarios = await data.json(); } catch {}
check(data.status === 200 && Object.keys(scenarios).length === 7, `scenario.json -> ${data.status}, ${Object.keys(scenarios).length} scenarios`);
check(JSON.stringify(scenarios.default?.route?.waypoints) === JSON.stringify([[2, 5, 2.9], [3.4, 5, 2.9], [3.85, 5, 2.45], [4.75, 5, 2.45], [5.2, 5, 2.9], [7, 5, 2.9]]), "default route in scenario.json is the documented one");

const pdf = await fetch(base + "/MEP-Coordination-Spike.pdf");
check(pdf.status === 200 && (pdf.headers.get("content-type") || "").includes("pdf"), `PDF -> ${pdf.status} ${pdf.headers.get("content-type")}`);
const missing = await fetch(base + "/no-such-page-xyz");
check(missing.status === 404, `unknown path -> ${missing.status}`);

console.log(failures.length ? `\n${failures.length} failure(s)` : "\nall live checks passed");
process.exit(failures.length ? 1 : 0);
