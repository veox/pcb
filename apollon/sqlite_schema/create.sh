#!/usr/bin/env bash

SQL=schema.sql
OUT=test1.sqlite

rm -f $OUT
cat $SQL | sqlite3 -batch $OUT

