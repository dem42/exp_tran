 function [flat_ident, flat_exp] = tensor_svd(filename1,dirname)

%we flatten the tensor along identity (54 faces, 25female,29male) 
%and expression (7)
%then we compute the svd (so that X = U*S*V')
%of those two to get the left singular U
%then and we keep the S*V' and collect them into the core tensor(using
%kroenecker product maybe)

NUMBER_OF_FACES = 54;
NUMBER_OF_EXPRES = 7;

fid = fopen(filename1,'rt');
num = fscanf(fid,'%i')
%cell array identity-expression
if num ~= NUMBER_OF_FACES*NUMBER_OF_EXPRES
    disp('error num is wrong');
end
cell_a = cell(NUMBER_OF_FACES,NUMBER_OF_EXPRES);
%cell_a = {'aaa', 'bbb', 'ccc'};
%cell_a(2)
for j=1:NUMBER_OF_FACES
    for k=1:NUMBER_OF_EXPRES
        str = fscanf(fid,'%s',1);
        cell_a(j,k) = {str};
    end
end
cell_a(1,1)
fclose(fid);

filename_face = sprintf('%s/%s',dirname,cell_a{1,1});
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

%flat_ident = zeros(NUMBER_OF_FACES,NUMBER_OF_EXPRES*3*number);
%flat_exp = zeros(NUMBER_OF_EXPRES,NUMBER_OF_FACES*3*number);

ident = zeros(NUMBER_OF_FACES,3*number);
exp = zeros(NUMBER_OF_EXPRES,3*number);

%counter for expression,identity so that we can flatten exp
exp_cnt = 0;
id_cnt = 1;

for j=1:NUMBER_OF_FACES
    for k=1:NUMBER_OF_EXPRES
        
        filename_face = sprintf('%s/%s',dirname,cell_a{j,k});

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
        
        exp(k,:) = zeros(1,3*number);
        exp(k,:) = fscanf(fid, '%f', [1,3*number]);
        fclose(fid);
    end
    if j==1
        flat_exp = exp;
    else
        flat_exp = [flat_exp exp];
    end
end
% 
% for j=1:NUMBER_OF_EXPRES
%     for k=1:NUMBER_OF_FACES
%         
%         filename_face = sprintf('%s/%s',dirname,cell_a{k,j});
% 
%         disp(filename_face);
%     
%         fid = fopen(filename_face,'rt');
%         tline = fgetl(fid);
%         tline = fgetl(fid);
%         tline = fgetl(fid);
%         tline = fgetl(fid);
%     
%         str = fscanf(fid,'%s',1);
%         number = fscanf(fid,'%i');
%         str = fscanf(fid,'%s',1);
% 
%         we'll be making a big assumption here that they are all the same
%         ident(j,:) = zeros(1,3*number);
%         ident(j,:) = fscanf(fid, '%f', [1,3*number]);
%         fclose(fid);
%     end
%     if j==1
%         flat_ident = ident;
%     else
%         flat_ident = [flat_ident ident];
%     end
% end
size(flat_exp)
%size(flat_ident)

%[U1,S1,V1] = svd(flat_ident');
[U2,S2,V2] = svd(flat_exp);

save('u2','u2');

end