
clc;
clear;
close all


myCacheFolder = fullfile('D:','\SPM_THESIS_FINAL_BACKUP\CACHE');
myCodeGenFolder = fullfile('D:','\SPM_THESIS_FINAL_BACKUP\CodeGen');

Simulink.fileGenControl('set', 'CacheFolder', myCacheFolder, ...
   'CodeGenFolder', myCodeGenFolder, ...
   'CodeGenFolderStructure', ...
    Simulink.filegen.CodeGenFolderStructure.TargetEnvironmentSubfolder);

run('SPM_ASSEMBLY_DataFile.m');
open_system('Dynamic_Simulation_Final.slx');