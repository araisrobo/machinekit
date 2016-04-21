-- SQLite HOWTO:    http://www.tutorialspoint.com/sqlite
-- SQLite LANG:     http://www.sqlite.org/lang.html

DROP TABLE IF EXISTS tools;
CREATE TABLE tools
(
    tool_no INT PRIMARY KEY, 
    drill_rpm REAL,
    drill_feed REAL,
    drill_depth REAL,
    drill_compensate REAL,
    tap_rpm REAL,
    tap_depth REAL,
    tap_pitch REAL,
    tap_compensate REAL,
    csk_rpm REAL,
    csk_depth REAL,
    csk_init_h REAL,
    csk_speed REAL,
    csk_probe_speed REAL,
    probe_din REAL,
    probe_ain REAL,
    probe_type REAL,
    probe_cond REAL,
    probe_alvl REAL,
    probe_dist REAL,
    csk_compensate REAL
);
-- -- useful commands
-- .tables
-- .headers on
-- SELECT * FROM pads;
-- .schema
