from run_lap_validation import perception_trace_topics


def test_full_pointcloud_observers_are_disabled_by_default():
    assert perception_trace_topics(False) == ()


def test_trace_mode_observes_each_existing_pipeline_stage_once():
    topics = perception_trace_topics(True)
    assert len(topics) == 5
    assert len({topic for _, topic in topics}) == len(topics)
    assert ('raw', 'lidar/raw') not in topics
    assert topics[0] == ('extended', 'lidar/extended')
    assert topics[-1] == ('clusters', 'clusters')
