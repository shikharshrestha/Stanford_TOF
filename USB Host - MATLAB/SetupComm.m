% % Run Script to establish Comm with ARM MCU

clc;
clear all;
close all;

mcu = serial('COM4','BAUD',57600,'Timeout',1);
fopen(mcu);