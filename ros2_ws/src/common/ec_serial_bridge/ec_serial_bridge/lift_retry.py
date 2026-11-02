"""Ack-driven retry bookkeeping for the type 02 lift command.

The EC lead said on 2026-11-02 that 02 is an event command, not a periodic
stream: send it when the robot is already standing at the target position, then
wait for the EC's own ``F0`` acknowledgement (``type 02``, result 0) and resend
the same frame if it does not arrive inside a configurable window, 200 ms by
default.

This module holds only the decision logic, no clock and no serial port, so the
offline tests can drive it with an explicit ``now``.
"""

import math


ACTION_RESEND = 'resend'
ACTION_GIVE_UP = 'give_up'


class LiftCommandRetrier:
    """Track one outstanding type 02 frame and decide when to resend it.

    ``max_retries`` counts the *resends* after the first frame:
    ``0`` restores the plain single-shot behaviour, ``-1`` means keep retrying
    until an ack arrives.
    """

    def __init__(self, ack_timeout_s=0.2, max_retries=5):
        if not math.isfinite(ack_timeout_s) or ack_timeout_s <= 0:
            raise ValueError('ack_timeout_s must be finite and positive')
        if not isinstance(max_retries, int) or isinstance(max_retries, bool):
            raise ValueError('max_retries must be an int')
        if max_retries < -1:
            raise ValueError('max_retries must be -1 (unlimited) or >= 0')
        self.ack_timeout_s = float(ack_timeout_s)
        self.max_retries = max_retries
        self._direction = None
        self._deadline = None
        self._attempts = 0

    @property
    def pending(self):
        return self._direction is not None

    @property
    def direction(self):
        return self._direction

    @property
    def attempts(self):
        """Frames written so far, counting the first send as attempt 1."""
        return self._attempts if self.pending else 0

    @property
    def resends(self):
        return max(self._attempts - 1, 0) if self.pending else 0

    def arm(self, direction, now):
        """Record that a fresh 02 frame for ``direction`` has just been written."""
        self._direction = int(direction)
        self._deadline = now + self.ack_timeout_s
        self._attempts = 1

    def settle(self, command_type, accepted, now):
        """Feed an ``F0`` ack back in.

        Returns ``'ok'`` when an accepted lift ack closed the outstanding
        command, ``'rejected'`` when the EC refused it, and ``''`` when the ack
        was for something else or nothing is pending. A rejection clears the
        command too: resending a frame the EC already refused only repeats the
        refusal.
        """
        if not self.pending or command_type != 0x02:
            return ''
        self.clear()
        return 'ok' if accepted else 'rejected'

    def plan(self, now):
        """Return ``(action, direction)``; ``action`` is ``None`` while waiting."""
        if not self.pending or now < self._deadline:
            return None, None
        if self.max_retries >= 0 and self.resends >= self.max_retries:
            return ACTION_GIVE_UP, self._direction
        return ACTION_RESEND, self._direction

    def retried(self, now):
        """Record that the resend returned by :meth:`plan` has been written."""
        if not self.pending:
            return
        self._attempts += 1
        self._deadline = now + self.ack_timeout_s

    def clear(self):
        self._direction = None
        self._deadline = None
        self._attempts = 0
