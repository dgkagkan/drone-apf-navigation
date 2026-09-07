"use strict";

// Schedule from completion so slow responses never build a request queue.
function startPolling(task, intervalMs, timeoutMs = 3000, retryMs = 1000) {
  let stopped = false;
  let timer = null;
  let controller = null;

  async function run() {
    if (stopped) return;
    const started = performance.now();
    controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), timeoutMs);
    let delay = intervalMs;
    try {
      await task(controller.signal);
      delay = Math.max(0, intervalMs - (performance.now() - started));
    } catch (error) {
      delay = retryMs;
    } finally {
      clearTimeout(timeout);
      controller = null;
      if (!stopped) {
        timer = setTimeout(run, delay);
      }
    }
  }

  timer = setTimeout(run, 0);
  return () => {
    stopped = true;
    clearTimeout(timer);
    controller?.abort();
  };
}
