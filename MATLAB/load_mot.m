function [data,headers] = load_mot(fname)

fid = fopen(fname);
fopen(fid);
line = fgetl(fid); 
flag = strcmp('endheader',line);

while flag == 0
line = fgetl(fid);  % denotes end of header info
line = sscanf(line,'%s');
flag = strcmp('endheader',line);
end

line = fgetl(fid);  % column names
cnames = textscan(line,'%s');
headers = cnames{1,1};

count = 1;
while (~feof(fid))
    line = fgetl(fid);
    dd = textscan(line,'%f');
    data(count,:) = dd{1,1}';
    count = count + 1;
end
fclose(fid);