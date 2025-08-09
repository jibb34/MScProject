import sqlite3
import json
import osmium


def build_index(pbf_path, db_path="way_index.sqlite"):
    """Builds up a lookup table of the pbf map for O(1) lookup times for node to way indexing"""

    connection = sqlite3.connect(db_path)
    c = connection.cursor()

    c.executescript("""
        PRAGMA journal_mode=WAL;
        PRAGMA synchronous=OFF;
        CREATE TABLE IF NOT EXISTS edges(
            u INTEGER, v INTEGER, way_id INTEGER, dir INTEGER, seq INTEGER
        );
        CREATE INDEX IF NOT EXISTS idx_edges_uv ON edges(u,v);
        CREATE UNIQUE INDEX IF NOT EXISTS idx_edges_unique ON edges(u,v,way_id,dir,seq);

        CREATE TABLE IF NOT EXISTS ways(
            way_id INTEGER PRIMARY KEY, tags TEXT
        );""")

    class Handler(osmium.SimpleHandler):
        def way(self, w):
            if len(w.nodes) < 2:
                return
            tags = {t.k: t.v for t in w.tags}
            c.execute(
                "INSERT OR REPLACE INTO ways(way_id, tags) VALUES (?, ?)",
                (int(w.id), json.dumps(tags)))
            refs = [int(n.ref) for n in w.nodes]
            rows = []
            for i, (a, b) in enumerate(zip(refs, refs[1:])):
                # store only the forward direction from OSM ordering
                rows.append((a, b, int(w.id), +1, i))
                rows.append((b, a, int(w.id), -1, i))
            c.executemany(
                "INSERT OR IGNORE INTO edges VALUES (?,?,?,?,?)", rows)
    connection.execute("BEGIN")
    Handler().apply_file(pbf_path, locations=False)
    connection.commit()
    connection.close()


if __name__ == "__main__":
    build_index("./data/osrm_map/map.pbf")
