import sqlite3
import json
import osmium
from utils.gpx_utils import haversine_m


def build_index(pbf_path, db_path="way_index.sqlite"):
    """Builds up a lookup table of the pbf map for O(1) lookup times for node to way indexing"""

    connection = sqlite3.connect(db_path)
    c = connection.cursor()

    c.executescript("""
            PRAGMA journal_mode=WAL;
            PRAGMA synchronous=OFF;

            CREATE TABLE IF NOT EXISTS edges(
                u INTEGER,
                v INTEGER,
                way_id INTEGER,
                dir INTEGER,
                seq INTEGER,
                len_m REAL
            );
            CREATE INDEX IF NOT EXISTS idx_edges_uv ON edges(u,v);
            CREATE INDEX IF NOT EXISTS idx_edges_u ON edges(u);
            CREATE UNIQUE INDEX IF NOT EXISTS idx_edges_unique
                ON edges(u,v,way_id,dir,seq,len_m);

            CREATE TABLE IF NOT EXISTS ways(
                way_id INTEGER PRIMARY KEY,
                tags TEXT
            );
        """)

    class Handler(osmium.SimpleHandler):
        def way(self, w):
            if len(w.nodes) < 2:
                return
            tags = {t.k: t.v for t in w.tags}
            c.execute(
                "INSERT OR REPLACE INTO ways(way_id, tags) VALUES (?, ?)",
                (int(w.id), json.dumps(tags)))
            refs = []
            for n in w.nodes:
                if not n.location.valid():
                    return
                refs.append((int(n.ref), n.location.lat, n.location.lon))
            rows = []
            for i in range(len(refs)-1):
                u, ulat, ulon = refs[i]
                v, vlat, vlon = refs[i+1]
                # calculate distance between nodes
                d = haversine_m(ulat, ulon, vlat, vlon)
                # store only the forward direction from OSM ordering
                rows.append((u, v, int(w.id), +1, i, d))
                rows.append((v, u, int(w.id), -1, i, d))
            c.executemany(
                "INSERT OR IGNORE INTO edges(u,v,way_id,dir,seq,len_m) VALUES (?,?,?,?,?,?)", rows)
    connection.execute("BEGIN")
    Handler().apply_file(pbf_path, locations=True)
    connection.commit()
    connection.close()


if __name__ == "__main__":
    build_index("./data/osrm_map/map.pbf")
