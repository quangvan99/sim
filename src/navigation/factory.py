"""Select a navigation backend from the process environment."""

from __future__ import annotations

import os

from navigation.adapter import NavigationAdapter
from navigation.custom_adapter import CustomNavigationAdapter
from navigation.nav2_adapter import Nav2NavigationAdapter
from navigation.navigator import Navigator
from transport import AMRTransport


def create_navigation_adapter(transport: AMRTransport) -> NavigationAdapter:
    """Build the configured backend while keeping custom as the safe default."""

    backend = os.environ.get("NAVIGATION_BACKEND", "custom").strip().lower()
    if backend == "custom":
        return CustomNavigationAdapter(Navigator(transport))
    if backend == "nav2":
        return Nav2NavigationAdapter(
            transport,
            base_url=os.environ.get("NAV2_GATEWAY_URL", "http://127.0.0.1:8010"),
            timeout_seconds=float(os.environ.get("NAV2_GATEWAY_TIMEOUT", "10")),
        )
    raise ValueError(
        f"Unsupported NAVIGATION_BACKEND={backend!r}; expected 'custom' or 'nav2'"
    )
