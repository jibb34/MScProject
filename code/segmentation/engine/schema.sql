CREATE TABLE IF NOT EXISTS segment_defs (
  segment_uid  BINARY(32) PRIMARY KEY,
  direction    ENUM('F') NOT NULL DEFAULT 'F',
  srid         INT NOT NULL DEFAULT 4326,
  point_count  INT UNSIGNED NOT NULL,
  coords_json  JSON NOT NULL,
  bbox_min_lat DOUBLE NOT NULL, bbox_min_lon DOUBLE NOT NULL,
  bbox_max_lat DOUBLE NOT NULL, bbox_max_lon DOUBLE NOT NULL,
  length_m     DOUBLE NOT NULL,
  created_at   TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
  KEY k_bbox_lat (bbox_min_lat, bbox_max_lat)
  KEY k_bbox_lon (bbox_min_lon, bbox_max_lon)
) ENGINE=InnoDB;
CREATE TABLE IF NOT EXISTS route_segments (
  segment_id   BIGINT UNSIGNED PRIMARY KEY AUTO_INCREMENT,
  segment_uid  BINARY(32) NOT NULL,
  route_id     BIGINT UNSIGNED NOT NULL,
  start_idx    INT UNSIGNED NOT NULL,
  end_idx      INT UNSIGNED NOT NULL,
  created_at   TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  UNIQUE KEY uq_span(route_id,start_idx,end_idx),
  KEY k_route(route_id),
  CONSTRAINT fk_def FOREIGN KEY (segment_uid) REFERENCES segment_defs(segment_uid)
    ON DELETE RESTRICT
) ENGINE=InnoDB;
CREATE TABLE IF NOT EXISTS segment_way_runs (
  segment_id    BIGINT UNSIGNED NOT NULL,
  seq           INT UNSIGNED NOT NULL,
  way_id        BIGINT NULL,
  run_start_idx INT UNSIGNED NOT NULL,
  run_end_idx   INT UNSIGNED NOT NULL,
  PRIMARY KEY (segment_id, seq),
  KEY k_way(way_id),
  CONSTRAINT fk_seg FOREIGN KEY (segment_id) REFERENCES route_segments(segment_id)
    ON DELETE CASCADE
) ENGINE=InnoDB;

