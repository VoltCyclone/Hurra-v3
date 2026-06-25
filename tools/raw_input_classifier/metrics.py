"""Separability metrics — pure stdlib so the headline number is torch-free testable.

AUC convention for this tool: label 1 = synthetic (the class the CNN tries to spot).
AUC 0.5 => classifier guessing => firmware WINS; AUC 1.0 => fully exposed.
"""


def auc(scores, labels):
    """Mann–Whitney U (rank) AUC with tie handling. Returns 0.5 if a class is empty."""
    n = len(scores)
    pos = sum(1 for l in labels if l == 1)
    neg = n - pos
    if pos == 0 or neg == 0:
        return 0.5
    order = sorted(range(n), key=lambda i: scores[i])
    ranks = [0.0] * n
    i = 0
    while i < n:
        j = i
        while j + 1 < n and scores[order[j + 1]] == scores[order[i]]:
            j += 1
        avg_rank = (i + j) / 2.0 + 1.0          # 1-based average rank over the tie group
        for k in range(i, j + 1):
            ranks[order[k]] = avg_rank
        i = j + 1
    sum_pos = sum(ranks[i] for i in range(n) if labels[i] == 1)
    u = sum_pos - pos * (pos + 1) / 2.0
    return u / (pos * neg)


def eer(scores, labels):
    """Equal-error rate: sweep thresholds, return the error where FPR ~= FNR."""
    pos = sum(1 for l in labels if l == 1)
    neg = len(labels) - pos
    if pos == 0 or neg == 0:
        return 0.5
    thresholds = sorted(set(scores))
    best = 1.0
    for t in thresholds + [thresholds[-1] + 1.0]:
        # predict 1 if score >= t
        fp = sum(1 for s, l in zip(scores, labels) if l == 0 and s >= t)
        fn = sum(1 for s, l in zip(scores, labels) if l == 1 and s < t)
        fpr = fp / neg
        fnr = fn / pos
        gap = abs(fpr - fnr)
        err = (fpr + fnr) / 2.0
        if gap <= best:                          # closest balance so far
            best = gap
            eer_val = err
    return eer_val
