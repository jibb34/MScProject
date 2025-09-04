-- Schema for storing reproducible, directional route segments and their
-- mapping to concrete routes.  This script is executed automatically
-- when the MySQL container starts for the first time via the
-- docker‑compose configuration in this directory.

-- Create the database if it does not already exist.  The name is
-- provided via the MYSQL_DATABASE variable in .env.  If you change
-- that value you should also update this script accordingly.
CREATE DATABASE IF NOT EXISTS routeseg CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci;
USE routeseg;

-- Table of unique segment definitions.  Each row represents a
-- reproducible segment geometry in a specific traversal direction.
-- Direction is stored as an ENUM; currently only 'F' (forward) is
-- supported by the engine.  The primary key is a 32‑byte binary
-- fingerprint (SHA‑256) computed from the rounded coordinate list
-- and the direction.  Coarse bounding box fields and the total
-- length in metres facilitate spatial indexing and quick queries.
CREATE TABLE IF NOT EXISTS segment_defs (
  segment_uid   BINARY(32) PRIMARY KEY,
  direction     ENUM('F') NOT NULL DEFAULT 'F',
  srid          INT NOT NULL DEFAULT 4326,
  point_count   INT UNSIGNED NOT NULL,
  coords_json   JSON NOT NULL,
  bbox_min_lat  DOUBLE NOT NULL,
  bbox_min_lon  DOUBLE NOT NULL,
  bbox_max_lat  DOUBLE NOT NULL,
  bbox_max_lon  DOUBLE NOT NULL,
  length_m      DOUBLE NOT NULL,
  created_at    TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  KEY k_bbox_lat (bbox_min_lat, bbox_max_lat),
  KEY k_bbox_lon (bbox_min_lon, bbox_max_lon)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- Instances of segments on a particular route.  Each row records
-- where a segment appears within the route by referencing the
-- parent route identifier (to be supplied by the caller) and the
-- inclusive start and exclusive end indices into the original
-- route points.  A unique constraint prevents duplicate span
-- definitions for the same route.
CREATE TABLE IF NOT EXISTS route_segments (
  segment_id   BIGINT UNSIGNED PRIMARY KEY AUTO_INCREMENT,
  segment_uid  BINARY(32) NOT NULL,
  route_id     BIGINT UNSIGNED NOT NULL,
  start_idx    INT UNSIGNED NOT NULL,
  end_idx      INT UNSIGNED NOT NULL,
  created_at   TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  UNIQUE KEY uq_span (route_id, start_idx, end_idx),
  KEY k_route (route_id),
  CONSTRAINT fk_def FOREIGN KEY (segment_uid)
    REFERENCES segment_defs(segment_uid) ON DELETE RESTRICT
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- Contiguous runs of identical way identifiers within a segment.
-- These runs preserve the order of ways along the route and record
-- the global index span in the original route.  A primary key on
-- (segment_id, seq) enforces ordering and prevents duplicates.
CREATE TABLE IF NOT EXISTS segment_way_runs (
  segment_id    BIGINT UNSIGNED NOT NULL,
  seq           INT UNSIGNED NOT NULL,
  way_id        BIGINT NULL,
  run_start_idx INT UNSIGNED NOT NULL,
  run_end_idx   INT UNSIGNED NOT NULL,
  PRIMARY KEY (segment_id, seq),
  KEY k_way (way_id),
  CONSTRAINT fk_seg FOREIGN KEY (segment_id)
    REFERENCES route_segments(segment_id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;