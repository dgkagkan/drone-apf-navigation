from argparse import Namespace
import os
import sqlite3

from apf_optuna.optimizer import configure_sqlite_database, configure_worker_environment


def test_parallel_worker_environment_is_isolated(monkeypatch):
    monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
    monkeypatch.delenv('GZ_PARTITION', raising=False)
    args = Namespace(
        worker_id=2,
        base_domain_id=40,
        base_agent_port=9000,
        run_id='stress/test',
    )

    configure_worker_environment(args)

    assert os.environ['ROS_DOMAIN_ID'] == '42'
    assert os.environ['GZ_PARTITION'] == 'apf_optuna_stress_test_worker_2'


def test_sqlite_database_uses_wal_mode(tmp_path):
    database_path = tmp_path / 'study.db'

    configure_sqlite_database(database_path)

    with sqlite3.connect(database_path) as connection:
        journal_mode = connection.execute('PRAGMA journal_mode').fetchone()[0]
    assert journal_mode == 'wal'
