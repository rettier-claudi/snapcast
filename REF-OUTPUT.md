# snapclient with a reference output (fork of snapcast 0.31.0)

Branch `ref-output`. `--ref-soundcard <alsa pcm>` adds a second ALSA output that follows the
main output sample by sample, for the reference input of an echo canceller (e.g. the USB
playback of a ReSpeaker XVF3800 next to a HiFiBerry).

A second snapclient instance syncs to the server on its own: the offset between the two
outputs differs after every start and wobbles by several samples. Here the main ALSA player
hands each block it writes, with the time its first frame is heard, to `RefOutput`
(`client/player/ref_output.cpp`): mono mix, own centred windowed-sinc resampler (64 taps,
no added delay), and a PI controller that trims the resampling step so the reference is
heard at main time + `--ref-offset`. Sound cards on different clocks are fine.

Options: `--ref-soundcard`, `--ref-rate` (16000), `--ref-channels` (2, all get the mono mix),
`--ref-offset` (us, 0), `--ref-buffer` (ms, 40). Log tag `RefOutput`
(`--logfilter "*:info,RefOutput:debug"` for details). ALSA player only.
