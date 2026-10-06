"""Factory for the Tilburg Hand, registered as an `rcs.hands` entry point."""

from rcs._core.common import Hand, HandConfig
from rcs_tilburg_hand.hand import THConfig, TilburgHand


def create_hand(cfg: HandConfig) -> Hand:
    if not isinstance(cfg, THConfig):
        msg = f"Expected THConfig for tilburg hand, got {type(cfg).__name__}"
        raise TypeError(msg)
    return TilburgHand(cfg, verbose=cfg.verbose)
