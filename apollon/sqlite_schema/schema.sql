--this is going to be a database, where design data is stored of placed
--components. Data like placement angle, coordinates, etc. This is an
--extraction of the PCB and schematic file. One db per board.

--So we load with data when update_dbsym is run, and after when the PCB is
--complete.

--Shall we populate other data from the main db? Like symbol, footprint,
--cad_model? So we don't have to query all this stuff again.

--The goal is to generate the CAD model of the complete board. Maybe we
--shall put another table with board data, like thinkness, etc.

--This is going to be a new PCB file format. :-)

BEGIN TRANSACTION;

--layout object, type 0
CREATE TABLE pcb (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	name TEXT
	--add pcb related attributes. DRC rules, etc.
);

--component object, type 1
CREATE TABLE component (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	refdes TEXT,
	value TEXT,
	dbsym_id INTEGER,
	mech_model TEXT
	--add here any other component level attribute
);

--padstack opbject, type 2
CREATE TABLE padstack (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	name TEXT
);

--primitive graphical oval object, type 3
CREATE TABLE oval (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	DX INTEGER,
	DY INTEGER,
	name TEXT
);

--primitive graphical arc object, type 4
CREATE TABLE arc (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	start_angle INTEGER,
	stop_angle INTEGER,
	R INTEGER,
	W INTEGER,
	name TEXT
);

--primitive graphical rectangle object, type 5
CREATE TABLE rectangle (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	X INTEGER,
	Y INTEGER,
	name TEXT
);

--primitive graphical line object, type 6
CREATE TABLE line (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	W INTEGER,
	name TEXT
);

--primitive graphical polygon object, type 7
CREATE TABLE polygon (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	name TEXT
);

--primitive round hole object, type 9
CREATE TABLE hole (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	D INTEGER,
	start_layer_id INTEGER,
	end_layer_id INTEGER,
	is_plated INTEGER,
	name TEXT
);

--primitive vertex object, type 8
CREATE TABLE vertex (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	X INTEGER,
	Y INTEGER,
	sequence INTEGER,
	name TEXT
);

--attribute object, type 11
CREATE TABLE attribute (
	id INTEGER PRIMARY KEY,
	name TEXT,
	value TEXT
);

--layer table
CREATE TABLE layer (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	material TEXT,
	Z INTEGER, --thickness
	is_outer INTEGER,
	is_mirrored INTEGER,
	drc INTEGER, --drc preference. Conductive DRC, non-conductive DRC, no DRC at all, etc.
	name TEXT
);

--object relations
CREATE TABLE object_relation (
	parent_type INTEGER,
	parent_id INTEGER,
	object_type INTEGER,
	object_id INTEGER,
	X INTEGER, --relative to the parent object.
	Y INTEGER, --relative to the parent object.
	ANG INTEGER, --relative to the parent object. Placement angle.
	layer_id INTEGER
);

--layer structure

--conductive layers from 0 to 31
--bottom is 0, top is 31

INSERT INTO layer (id, is_outer, is_mirrored, name) VALUES (0, 1, 0, "bottom");
INSERT INTO layer (id, is_outer, is_mirrored, name) VALUES (31, 1, 1, "top");

--silk layers from 32 to 63
--bottom is 32, top is 63

INSERT INTO layer (id, is_outer, is_mirrored, name) VALUES (32, 1, 0, "bottom silk");
INSERT INTO layer (id, is_outer, is_mirrored, name) VALUES (63, 1, 1, "top silk");

--solder mask layers from 64 to 95
--bottom is 64, top is 95
--others make no sense, but I leave it there.

INSERT INTO layer (id, is_outer, is_mirrored, name) VALUES (64, 1, 0, "bottom mask");
INSERT INTO layer (id, is_outer, is_mirrored, name) VALUES (95, 1, 1, "top mask");

--solder paste layers from 96 to 127
--bottom is 96, top is 127
--others make no sense, but I leave it there.

INSERT INTO layer (id, is_outer, is_mirrored, name) VALUES (96, 1, 0, "bottom mask");
INSERT INTO layer (id, is_outer, is_mirrored, name) VALUES (127, 1, 1, "top mask");

COMMIT;

