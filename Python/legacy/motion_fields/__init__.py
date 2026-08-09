"""Motion Fields for Interactive Character Animation (Lee et al. 2010).

Headless port of ``Motion Fields For Interactive Character Animation.ipynb``:
pose algebra, similarity metric, GPU k-NN, transition precomputation, and fitted
value function training. All rendering and interactive control is omitted.

Run it with::

    python motion_fields_main.py
    python -m motion_fields
"""

from .config import Config

__all__ = ['Config']
