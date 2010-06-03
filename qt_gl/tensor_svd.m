function [flat_ident, flat_exp, U1, U2] = tensor_svd(filename1,dirname)

%we flatten the tensor along identity (54 faces, 25female,29male) 
%and expression (7)
%then we compute the svd (so that X = U*S*V')
%of those two to get the left singular U
%then and we keep the S*V' and collect them into the core tensor(using
%kroenecker product maybe)

const_NUMBER_OF_FACES = 54;

fid = fopen(filename1,'rt');
num = fscanf(fid,'%i')
cell_a = cell(1,num);
%cell_a = {'aaa', 'bbb', 'ccc'};
%cell_a(2)
for j=1:num
    str = fscanf(fid,'%s',1);
    cell_a(j) = {str};    
end
cell_a(1)
fclose(fid);

filename_face = sprintf('%s/%s',dirname,cell_a{1});
disp(filename_face);
fid = fopen(filename_face,'rt');
tline = fgetl(fid);
tline = fgetl(fid);
tline = fgetl(fid);
tline = fgetl(fid);
disp(tline);
str = fscanf(fid,'%s',1);
number = fscanf(fid,'%i');

fclose(fid);

flat_ident = zeros(num,3*number);
flat_exp = zeros(num,3*number);

%counter for expression,identity so that we can flatten exp
exp_cnt = 0;
id_cnt = 1;

for j=1:num
    filename_face = sprintf('%s/%s',dirname,cell_a{j});

    disp(filename_face);
    
    fid = fopen(filename_face,'rt');
    tline = fgetl(fid);
    tline = fgetl(fid);
    tline = fgetl(fid);
    tline = fgetl(fid);
    
    str = fscanf(fid,'%s',1);
    number = fscanf(fid,'%i');
    str = fscanf(fid,'%s',1);

    %we'll be making a big assumption here that they are all the same

    flat_ident(j,:) = zeros(1,3*number);
    flat_exp(exp_cnt*const_NUMBER_OF_FACES + id_cnt,:) = zeros(1,3*number);

    flat_iden(j,:) = fscanf(fid, '%f', [1,3*number]);
    flat_exp(exp_cnt*const_NUMBER_OF_FACES + id_cnt,:) = flat_iden(j,:);
    
    exp_cnt = exp_cnt + 1;
    if exp_cnt == 7
        exp_cnt = 0;
        id_cnt = id_cnt+1;
    end

    fclose(fid);
end

exp_cnt
id_cnt

[U1,S1,V1] = svd(flat_ident');
[U2,S2,V2] = svd(flat_exp');

end