const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");
const test = require("node:test");
const { setTimeout: sleep } = require("node:timers/promises");

const startPolling = vm.runInNewContext(
  fs.readFileSync(path.join(__dirname, "../web/polling.js"), "utf8") + "\nstartPolling;",
  { setTimeout, clearTimeout, performance, AbortController },
);

test("a slow response never overlaps the next request", async () => {
  let calls = 0;
  let finish;
  const pending = new Promise(resolve => { finish = resolve; });
  const stop = startPolling(async () => { calls++; await pending; }, 5, 1000);
  try {
    await sleep(80);
    assert.equal(calls, 1);
    finish();
    await sleep(30);
    assert.ok(calls > 1);
  } finally {
    stop();
    finish();
  }
});

test("timeout aborts a stalled request and polling recovers", async () => {
  let calls = 0;
  let aborted = false;
  const stop = startPolling(async signal => {
    calls++;
    if (calls !== 1) return;
    await new Promise((resolve, reject) => {
      signal.addEventListener("abort", () => {
        aborted = true;
        reject(new Error("timeout"));
      }, { once: true });
    });
  }, 5, 25, 10);
  try {
    await sleep(100);
    assert.ok(aborted);
    assert.ok(calls > 1);
  } finally {
    stop();
  }
});

test("failed requests back off before retrying", async () => {
  let calls = 0;
  const stop = startPolling(async () => { calls++; throw new Error("503"); }, 5, 1000, 80);
  try {
    await sleep(30);
    assert.equal(calls, 1);
    await sleep(90);
    assert.equal(calls, 2);
  } finally {
    stop();
  }
});

test("stopping aborts an in-flight request and prevents rescheduling", async () => {
  let calls = 0;
  let aborted = false;
  const stop = startPolling(async signal => {
    calls++;
    await new Promise((resolve, reject) => {
      signal.addEventListener("abort", () => {
        aborted = true;
        reject(new Error("stopped"));
      }, { once: true });
    });
  }, 5, 1000);
  try {
    await sleep(20);
    stop();
    await sleep(30);
    assert.ok(aborted);
    assert.equal(calls, 1);
  } finally {
    stop();
  }
});
