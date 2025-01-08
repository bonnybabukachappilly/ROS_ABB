ROBOT_INFO_MAPPING: dict[str, str] = {
    "major": "version_major",
    "minor": "version_minor",
    "build": "build",
    "revision": "revision",
    "sub-revision": "sub_revision",
    "buildtag": "build_tag",
    "title": "ctrl_title",
    "type": "ctrl_type",
    "description": "description",
    "date": "date",
    "name": "name",
    "rwversion": "rw_version",
    "sysid": "sysid",
    "starttm": "start_time",
    "rwversionname": "rw_version_name",
    "rwdistributionversion": "rw_distribution_version",
}

ROBOT_ELOG_KEYS: dict[str, str] = {
    "_type": "elog_type",
    "title": "elog_title",
    "msgtype": "elog_msgtype",
    "code": "elog_code",
    "tstamp": "elog_tstamp",
    "desc": "elog_desc",
    "conseqs": "elog_conseqs",
    "causes": "elog_causes",
    "actions": "elog_actions",
    "argv": "elog_others",
}

ROBOT_ELOG_TYPE: dict[str, str] = {
    "1": "INFO",
    "2": "WARN",
    "3": "ERROR",
}


ROBOT_ENERGY: dict[str, str] = {
    "_title": "energy_title",
    "state": "energy_state",
    "energy-state": "energy_state",
    "change-count": "energy_change_count",
    "time-stamp": "energy_time_stamp",
    "reset-time": "energy_reset-time",
    "interval-length": "energy_interval_length",
    "interval-energy": "energy_interval",
    "accumulated-energy": "energy_accumulated",
}

ROBOT_JOINT_TARGET: dict[str, str] = {
    "rax_1": "joint_1",
    "rax_2": "joint_2",
    "rax_3": "joint_3",
    "rax_4": "joint_4",
    "rax_5": "joint_5",
    "rax_6": "joint_6",
}
