import os, sys, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
from raw_input_classifier import metrics


class MetricsTest(unittest.TestCase):
    def test_perfect_separation_auc_1(self):
        scores = [0.1, 0.2, 0.3, 0.8, 0.9, 1.0]
        labels = [0, 0, 0, 1, 1, 1]
        self.assertAlmostEqual(metrics.auc(scores, labels), 1.0, places=6)

    def test_inverted_auc_0(self):
        scores = [0.9, 0.8, 0.7, 0.2, 0.1, 0.0]
        labels = [0, 0, 0, 1, 1, 1]
        self.assertAlmostEqual(metrics.auc(scores, labels), 0.0, places=6)

    def test_identical_scores_auc_half(self):
        scores = [0.5] * 6
        labels = [0, 1, 0, 1, 0, 1]
        self.assertAlmostEqual(metrics.auc(scores, labels), 0.5, places=6)

    def test_eer_perfect_is_zero(self):
        scores = [0.1, 0.2, 0.8, 0.9]
        labels = [0, 0, 1, 1]
        self.assertLess(metrics.eer(scores, labels), 1e-6)

    def test_eer_random_near_half(self):
        scores = [0.5, 0.5, 0.5, 0.5]
        labels = [0, 1, 0, 1]
        self.assertGreater(metrics.eer(scores, labels), 0.4)


if __name__ == "__main__":
    unittest.main()
