import numpy as np

from robot_manipulation.data.dataset_builder import Episode, load_episode, save_episode
from robot_manipulation.data.rerun_logger import RerunLogger


def test_rerun_logger_writes_rrd_without_error(tmp_path):
    logger = RerunLogger("test_ep", out_dir=str(tmp_path))
    for t in range(3):
        logger.log_step(t, {"proprio": np.zeros(6, dtype=np.float32)}, np.zeros(3, dtype=np.float32))
    path = logger.finalize(True)
    assert (tmp_path / "test_ep.rrd").exists()
    assert path == str(tmp_path / "test_ep.rrd")


def test_episode_npz_roundtrip(tmp_path):
    # rerun-sdk 0.42 has no public .rrd read-back API, so the real training
    # dataset round-trips through .npz, not through the .rrd file above.
    episode = Episode(
        proprio=np.random.randn(10, 6).astype(np.float32),
        action=np.random.randn(10, 3).astype(np.float32),
        success=True,
    )
    path = save_episode(episode, str(tmp_path), "test_ep")
    loaded = load_episode(path)
    np.testing.assert_array_equal(loaded.proprio, episode.proprio)
    np.testing.assert_array_equal(loaded.action, episode.action)
    assert loaded.success == episode.success
