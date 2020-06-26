
%%Exemplary location for the scripts

system('C:\Users\Biomechatronic\Desktop\GUIs\General\update_ProgramsGeneral.bat')
    system('C:\Users\Biomechatronic\Desktop\GUIs\General\makeGeneral.bat')
    disp("Waiting..");
    pause(1);
    disp("Running..");
    system('C:\Users\Biomechatronic\Desktop\GUIs\General\runGeneral.bat') 

    filename = 'C:\Users\Biomechatronic\Desktop\GUIs\General\Receiver\General.log'; %creates a log file to record the data
    delimiterIn = ' ';
    headerlinesIn = 1;
    dataTemp = importdata(filename,delimiterIn,headerlinesIn);
    dataTemp = dataTemp.data;
delimiterIn = ' ';
headerlinesIn = 1;
dataTemp = importdata(filename,delimiterIn,headerlinesIn);
dataTemp = dataTemp.data;

 %% different amounts of slots can be added 
    %for different parameters of interest given that they are defined in
    %the SEACont.c script

field1 = 'time';  value1 = dataTemp(:,1);
field2 = 'torsion';  value2 = dataTemp(:,2);
field3 = 'motorAngle';  value3 = dataTemp(:,3);
field4 = 'Reference';  value4 = dataTemp(:,4);
field5 = 'inputSignal';  value5 = dataTemp(:,5);
field6 = 'sigma';  value6 = dataTemp(:,6);

Experiment = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,field6,value6);

for i = 1:length(Experiment.time)-1

   difsi(i) = Experiment.time(i+1)-Experiment.time(i);

end

Ts = mean(difsi);
Ref = timeseries(Experiment.Reference,Experiment.time);
Qm = timeseries(Experiment.motorAngle,Experiment.time);
Qd = timeseries(Experiment.torsion,Experiment.time);
Tm = timeseries(Experiment.inputSignal,Experiment.time);
Sigma = timeseries(Experiment.sigma,Experiment.time);

 figure

plot(Ref)
hold on
plot(Qd*5000)
legend('Reference','Torsion')
