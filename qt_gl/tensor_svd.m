function out = tensor_svd(filename1,filename2)

fid = fopen(filename1,'rt');
num = fscanf(fid,'%i')
cell_a = cell(1,num);
%cell_a = {'aaa', 'bbb', 'ccc'};
%cell_a(2)
for j=1:num
    str = fscanf(fid,'%s',1);
    cell_a(j) = {str};    
end
cell_a(1000)

ten = zeros(1,10);

fid = fopen(filename2,'rt');
tline = fgetl(fid);
tline = fgetl(fid);
tline = fgetl(fid);
tline = fgetl(fid);
disp(tline);
str = fscanf(fid,'%s',1)
num = fscanf(fid,'%i')
str = fscanf(fid,'%s',1)
ten = fscanf(fid, '%i', [1,10])

fclose(fid);

end