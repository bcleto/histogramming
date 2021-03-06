%top_v10.bit
clear
clc
addpath('./commands');

%%% Open FPGA and download bitfile
dev = openFPGA;
dev 


bitfile = '';
configureFPGA(dev, bitfile)

mask = 4294967295; %2^32-1

ep04value=uint32(3); %accumulation
setwireinvalue(dev,uint32(hex2dec('04')),ep04value,mask);
updatewireins(dev);

ep05value=uint32(hex2dec('00000001'));% reset = 1
setwireinvalue(dev,uint32(hex2dec('05')),ep05value,mask);
updatewireins(dev);

pause(1);
ep05value=uint32(hex2dec('00000002'));% start = 1 , reset = 0
setwireinvalue(dev,uint32(hex2dec('05')),ep05value,mask);
updatewireins(dev); 
pause(1);

ep06value=uint32(255); %input data LSB 32 bits
setwireinvalue(dev,uint32(hex2dec('06')),ep06value,mask);
updatewireins(dev);
pause(1);

ep40value = int32(hex2dec('1'));%input valid
activatetriggerin(dev,int32(hex2dec('40')), ep40value);


pause(0.1);
updatewireouts(dev);
output_fifo_data_number=getwireoutvalue(dev,uint32(hex2dec('20')))

%addr0 -4*(accumulation)


%% read output fifos
fifo_block_read = 1024; %max 1024
fifo_block_size =1024*1;    %total bit size = fifo_block_size/4
output =readfromblockpipeout(dev,uint32(hex2dec('A0')),fifo_block_read,fifo_block_size ); 
output2 =readfromblockpipeout(dev,uint32(hex2dec('A1')),fifo_block_read,fifo_block_size ); 

fifo_depth=fifo_block_size;
fifo_byte = fifo_depth/4;

output_hex = dec2hex(output,2);
cat_output = char(zeros(fifo_byte,8));
bincat_output = (zeros(fifo_byte,32));
output_hex2 = dec2hex(output2,2);
cat_output2 = char(zeros(fifo_byte,8));
bincat_output2 = (zeros(fifo_byte,32));

for i=1:4:fifo_depth
    cat_output((i+3)/4,:) = strcat(output_hex(i+3,1),output_hex(i+3,2),output_hex(i+2,1),output_hex(i+2,2),output_hex(i+1,1),output_hex(i+1,2),output_hex(i,1),output_hex(i,2));
    bincat_output((i+3)/4,:)=(hexToBinaryVector(cat_output((i+3)/4,:),32));
    cat_output2((i+3)/4,:) = strcat(output_hex2(i+3,1),output_hex2(i+3,2),output_hex2(i+2,1),output_hex2(i+2,2),output_hex2(i+1,1),output_hex2(i+1,2),output_hex2(i,1),output_hex2(i,2));
    bincat_output2((i+3)/4,:)=(hexToBinaryVector(cat_output2((i+3)/4,:),32));
end

%output in 32 bits
freq = zeros(fifo_byte,1);
mode = zeros(fifo_byte,1);
for i=1:fifo_byte
freq(i)=bi2de(bincat_output(i,1:32));
mode(i)=bi2de(bincat_output2(i,1:32));
end


%% write and read
fifo_block_read = 1024; %max 1024
fifo_block_size =1024*1;    %total bit size = fifo_block_size/4
val = [1 2 2 4 5];

output =writetopipein(dev,uint32(hex2dec('80')),val,fifo_block_size ); 



output2 =readfromblockpipeout(dev,uint32(hex2dec('A1')),fifo_block_read,fifo_block_size ); 

fifo_depth=fifo_block_size;
fifo_byte = fifo_depth/4;


output_hex2 = dec2hex(output2,2);
cat_output2 = char(zeros(fifo_byte,8));
bincat_output2 = (zeros(fifo_byte,32));

for i=1:4:fifo_depth
    
    cat_output2((i+3)/4,:) = strcat(output_hex2(i+3,1),output_hex2(i+3,2),output_hex2(i+2,1),output_hex2(i+2,2),output_hex2(i+1,1),output_hex2(i+1,2),output_hex2(i,1),output_hex2(i,2));
    bincat_output2((i+3)/4,:)=(hexToBinaryVector(cat_output2((i+3)/4,:),32));
end

%output in 32 bits

in_out = zeros(fifo_byte,1);
for i=1:fifo_byte
in_out(i)=bi2de(bincat_output2(i,1:32));
end












