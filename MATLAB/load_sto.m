function [data,headers] = load_sto(fname)
fid = fopen(fname);
fopen(fid);
line = fgetl(fid);  % name of files
line = fgetl(fid);  % version
line = fgetl(fid);  % number of rows
line = fgetl(fid);  % number of columns
n_columns = textscan(line,'nColumns=%f');
line = fgetl(fid);  % rotation in deg or radians
line = fgetl(fid);  % denotes end of header info
flag = strcmp('endheader',line);
while flag == 0
line = fgetl(fid);  % denotes end of header info
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