#!/bin/bash

# Small script to auto compilation and database removal.
rm -rf rtdb.db
g++ -o binary main.cpp helpers/Helpers.cpp database/DatabaseManager.cpp queue/VarDisQueue.cpp protocol/VarDisProtocol.cpp -ltins -ltbb
