#!/bin/bash
sudo service wheel stop
sudo service servo stop
sudo service joy stop
sudo service light stop
sudo service wheel start
sudo service servo start
sudo service joy start
sudo service light start