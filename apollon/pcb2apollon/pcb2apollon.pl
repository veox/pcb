#!/usr/bin/env perl

use strict;
use IO::File;
use DBI;

my $pcb_name = shift;
my $db_filename = shift;
my $pcb_file;

my $date=localtime();

my %null_point=pcb_get_null_coordinates($pcb_name);

$pcb_file=new IO::File($pcb_name, "r") or die "Can't open pcb file.\n";

my $dbh = DBI->connect("dbi:SQLite:dbname=$db_filename","","");

if ($dbh==undef) {
	die "Can't connect to database.\n";
}

my $pcb=insert_pcb_data($dbh, $pcb_name);

while (my $line = $pcb_file->getline()) {
	if ($line=~/Element\["(.*)" "(.*)" "(.*)" "(.*)" (.+) (.+) (.+) (.+) (.+) (.+) "(.*)"\]/) {
		my $element_flags=$1;
		my $element_footprint=$2;
		my $element_RefDes=$3;
		my $element_value=$4;
		my $element_x=(pcb_parse_val_nm($5)-$null_point{x});
		my $element_y=($null_point{y}-pcb_parse_val_nm($6));

		print ("Element $element_RefDes at $element_x, $element_y\n");

		while ($line = $pcb_file->getline()) {

			if ($line=~/Pin\[(.+) (.+) (.+) (.+) (.+) (.+) "(.*)" "(.*)" "(.*)"\]/) {
				my $pin_rX=pcb_parse_val_nm($1);
				my $pin_rY=pcb_parse_val_nm($2);
				my $pin_thickness=pcb_parse_val_nm($3);
				my $pin_clearence=$pin_thickness+pcb_parse_val_nm($4);
				my $pin_mask=pcb_parse_val_nm($5);
				my $pin_drill=pcb_parse_val_nm($6);
				print ("Pin at $pin_rX,$pin_rY\n");
			} elsif ($line=~/Pad\[(.+) (.+) (.+) (.+) (.+) (.+) (.+) "(.*)" "(.*)" "(.*)"\]/) {
				my $pad_rx1=pcb_parse_val_nm($1);
				my $pad_ry1=pcb_parse_val_nm($2);
				my $pad_rx2=pcb_parse_val_nm($3);
				my $pad_ry2=pcb_parse_val_nm($4);
				my $pad_thickness=pcb_parse_val_nm($5);
				my $pad_clearence=$pad_thickness+pcb_parse_val_nm($6);
				my $pad_mask=pcb_parse_val_nm($7);

				my $pad_x=($pad_rx2+$pad_rx1)/2;
				my $pad_y=($pad_ry2+$pad_ry1)/2;

				print ("Pad at $pad_x,$pad_y\n");
			} elsif ($line=~/\t\)/) {
				last;
			} else {
			}
		}
	} elsif ($line=~/Via\[(.+) (.+) (.+) (.+) (.+) (.+) "(.*)" "(.*)"\]/) {
		my $via_x=pcb_parse_val_nm($1);
		my $via_y=pcb_parse_val_nm($2);
		my $via_thickness=pcb_parse_val_nm($3);
		my $via_clearance=pcb_parse_val_nm($4);
		my $via_mask=pcb_parse_val_nm($5);
		my $via_drill=pcb_parse_val_nm($6);
		my $via_sflag=$6;
#		print ("Via at $via_x, $via_y, $via_mask\n");

#create a padstack from the Via.

		#check if have a condictive pad like this
		my $row = get_oval_data_by_size($dbh, $via_thickness, $via_thickness);
		if ($row==undef) {# error, or no such pad
			#let's insert
			insert_oval_data($dbh, $via_thickness, $via_thickness, "VIA_conductive");
			#get the id of the conductive pad
			$row = get_oval_data_by_size($dbh, $via_thickness, $via_thickness);
		}
		my $via_conductive_pad_id=$row->{'id'};

		#check if have a mask "pad" like this
		$row = get_oval_data_by_size($dbh, $via_mask, $via_mask);
		if ($row==undef) {# error, or no such pad
			#let's insert
			insert_oval_data($dbh, $via_mask, $via_mask, "VIA_mask");
			#get the id of the mask pad
			$row = get_oval_data_by_size($dbh, $via_mask, $via_mask);
		}
		my $via_mask_pad_id=$row->{'id'};

		#process the hole
		$row = get_hole_data($dbh, $via_drill, 0, 31, 1);
		if ($row==undef) {# error, or no such hole
			#let's insert
			insert_hole_data($dbh, $via_drill, 0, 31, 1, "VIA_hole");
			$row=get_hole_data($dbh, $via_drill, 0, 31, 1);
		}
		my $via_hole_id=$row->{'id'};

		my $padstackname = "VIA_C" . $via_conductive_pad_id . "_M" . $via_mask_pad_id . "_H" . $via_hole_id;
		$row=get_padstack_by_name($dbh, $padstackname);

		my $padstack;
		if ($row==undef) {
			$padstack=insert_padstack_data($dbh, $padstackname);
		} else {
			$padstack=$row->{'id'};
		}

		#add the relation. I.e. add the via to the pcb object.
		insert_relation($dbh, 0, $pcb, 2, $padstack, $via_x, $via_y, 0, 0);

	} else {
#		print ("Unknown data line '$line'\n");

	}

}

$dbh->disconnect();
close ($pcb_file);

sub pcb_parse_val_nm {

	my $value=shift;

	if ($value=~/(.+)mm$/) {
		return $1*1E6;
	} elsif ($value=~/(.+)mil$/) {
		return $1*0.0254*1E6;
	} elsif ($value=~/(.+)/) {
		return $1*0.000254*1E6;
	}
}

sub pcb_get_null_coordinates {
	my $pcb_name = shift;
	my $pcb_file;

	my %coords;
	$coords{x}=0;
	$coords{y}=0;

	unless (open ($pcb_file, "<", $pcb_name) ) {
		print STDERR "Could not open pcb file: '$pcb_name'\n";
		return %coords;
	}
	while(<$pcb_file>) {
		my $line=$_;
		if ($line=~/Element\["(.*)" "__REFPOINT__" "(.*)" "(.*)" (.+) (.+) (.+) (.+) (.+) (.+) "(.*)"\]/) {
			$coords{x}=pcb_parse_val_nm($4);
			$coords{y}=pcb_parse_val_nm($5);
			last;
		}
	}
	close ($pcb_file);
	return %coords;
}

sub get_oval_data_by_size {

	my $dbh=shift;
	my $X=shift;
	my $Y=shift;

	my $sth = $dbh->prepare("SELECT * FROM oval WHERE DX=$X AND DY=$Y;");
	$sth->execute();
	my $row = $sth->fetchrow_hashref;
	if (defined $sth){
		$sth->finish();
	}

	return $row;
}

sub insert_oval_data {

	my $dbh=shift;
	my $X=shift;
	my $Y=shift;
	my $name=shift;

	my $sth = $dbh->prepare("INSERT INTO oval (DX, DY, name) VALUES ($X, $Y, \"$name\");");
	my $ret=$sth->execute();
	if (defined $sth){
		$sth->finish();
	}

	return $ret;
}

sub get_hole_data {

	my $dbh=shift;
	my $D=shift;
	my $start_layer=shift;
	my $end_layer=shift;
	my $is_plated=shift;

	my $sth = $dbh->prepare("SELECT * FROM hole WHERE D=$D AND start_layer_id=$start_layer AND end_layer_id=$end_layer AND is_plated=$is_plated;");
	$sth->execute();
	my $row = $sth->fetchrow_hashref;
	if (defined $sth){
		$sth->finish();
	}
	return $row;

}

sub insert_hole_data {

	my $dbh=shift;
	my $D=shift;
	my $start_layer=shift;
	my $end_layer=shift;
	my $is_plated=shift;
	my $name=shift;

	my $sth = $dbh->prepare("INSERT INTO hole (D, start_layer_id, end_layer_id, is_plated, name) VALUES ($D, $start_layer, $end_layer, $is_plated, \"$name\");");

	my $ret=$sth->execute();
	if (defined $sth){
		$sth->finish();
	}

	return $ret;
}

sub get_parent_id {

	my $dbh=shift;
	my $parent_type=shift;
	my $type=shift;
	my $id=shift;

	my $sth = $dbh->prepare("SELECT * FROM object_relation WHERE parent_type=$parent_type AND object_type=$type AND object_id=$id;");

	$sth->execute();
	my $row = $sth->fetchrow_hashref;
	if (defined $sth){
		$sth->finish();
	}

	return $row;
}

sub insert_padstack_data {

	my $dbh=shift;
	my $name=shift;

	my $sth = $dbh->prepare("INSERT INTO padstack (name) VALUES (\"$name\");");

	$sth->execute();
	if (defined $sth){
		$sth->finish();
	}

	my $id = $dbh->last_insert_id("","","","");

	return $id;
}

sub insert_relation {

	my $dbh=shift;
	my $parent_type=shift;
	my $parent_id=shift;
	my $type=shift;
	my $id=shift;
	my $x=shift;
	my $y=shift;
	my $ang=shift;
	my $layer=shift;

	my $sth = $dbh->prepare("INSERT INTO object_relation (parent_type, parent_id, object_type, object_id, X, Y, ANG, layer_id) VALUES ($parent_type, $parent_id, $type, $id, $x, $y, $ang, $layer);");

	$sth->execute();
	if (defined $sth){
		$sth->finish();
	}

	$id = $dbh->last_insert_id("","","","");

	return $id;
}

sub get_padstack_by_name {
	my $dbh=shift;
	my $name=shift;

	my $sth = $dbh->prepare("SELECT id FROM padstack WHERE name=\"$name\";");

	$sth->execute();
	my $row = $sth->fetchrow_hashref;
	if (defined $sth){
		$sth->finish();
	}

	return $row;
}

sub insert_pcb_data {
	my $dbh=shift;
	my $name=shift;

	my $sth = $dbh->prepare("INSERT INTO pcb (name) VALUES (\"$name\");");

	$sth->execute();
	if (defined $sth){
		$sth->finish();
	}

	my $id = $dbh->last_insert_id("","","","");

	return $id;
}

