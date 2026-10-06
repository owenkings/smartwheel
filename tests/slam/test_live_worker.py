"""Concurrency guarantees of the bounded map rebuild worker."""

import threading
import time
import unittest

from wc_maps.live_map import LatestRebuildWorker


class LatestWorkerTests(unittest.TestCase):
    def test_only_latest_pending_revision_is_run_and_complete_older_result_is_retained(self):
        entered, release = threading.Event(), threading.Event()
        entered_latest, release_latest = threading.Event(), threading.Event()
        seen = []

        def build(revision):
            seen.append(revision)
            if revision == 1:
                entered.set()
                if not release.wait(5):
                    raise RuntimeError("test release timed out")
            if revision == 3:
                entered_latest.set()
                if not release_latest.wait(5):
                    raise RuntimeError("latest test release timed out")
            return {"revision": revision}

        worker = LatestRebuildWorker(build)
        try:
            worker.submit(1)
            self.assertTrue(entered.wait(5))
            worker.submit(2)
            last = worker.submit(3)
            release.set()
            self.assertTrue(entered_latest.wait(5))
            # Entering build 3 proves build 1 committed its complete result.
            self.assertEqual(worker.poll(), (1, {"revision": 1}, None))
            release_latest.set()
            result = None
            deadline = time.monotonic() + 5
            while result is None and time.monotonic() < deadline:
                result = worker.poll()
                if result is None:
                    time.sleep(0.01)
            self.assertEqual(seen, [1, 3])
            self.assertEqual(result, (last, {"revision": 3}, None))
            self.assertIsNone(worker.poll())
        finally:
            release.set()
            release_latest.set()
            self.assertTrue(worker.close())

    def test_failed_rebuild_has_no_partial_success_result(self):
        def fail(_):
            raise ValueError("raw frame index incomplete")

        worker = LatestRebuildWorker(fail)
        try:
            token = worker.submit(5)
            result = None
            deadline = time.monotonic() + 5
            while result is None and time.monotonic() < deadline:
                result = worker.poll()
                if result is None:
                    time.sleep(0.01)
            self.assertEqual(result, (token, None, "raw frame index incomplete"))
        finally:
            self.assertTrue(worker.close())


if __name__ == "__main__":
    unittest.main()
