clear all 
close all

rep=8;
goy=5;
startLine=1;
endLine=70;
x=[];
y=[];
count=0;

for i=1:rep
    count=count+1;
    x=[x startLine endLine];
    y=[y goy*count goy*count];
    count=count+1;    
    x=[x endLine startLine];
    y=[y goy*count goy*count];
    
end

plot (x,y)
axis equal