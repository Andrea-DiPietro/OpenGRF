function [data,headers,headersXYZ] = load_trc(fname)
fid = fopen(fname);
fopen(fid);
line = fgetl(fid);  %pathfiletype
line = fgetl(fid);  % Datarate
data1=split(line);
line = fgetl(fid);  %dati
datah=split(line);
line = fgetl(fid);  %intestazione
cnames = textscan(line,'%s');
headers = cnames{1,1};
headersXYZ=headers;
j=3;
for i=3:length(headers)
    headersXYZ{j,1}=strcat(headers{i,1},'_X');
    headersXYZ{j+1,1}=strcat(headers{i,1},'_Y');
    headersXYZ{j+2,1}=strcat(headers{i,1},'_Z');
    j=j+3;
end
line = fgetl(fid);  %x-y-z
%line = fgetl(fid);
count = 1;
while (~feof(fid))
    line = fgetl(fid);
    dd = textscan(line,'%f');
    data(count,:) = dd{1,1}';
    count = count + 1;
end
fclose(fid);
