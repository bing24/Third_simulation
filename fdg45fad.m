
clear

meet=[1;2;3;4];
judge=[];
for i=1:length(meet)-1
    judge(i)=max(meet(i:i+1))~=meet(i+1);
end
if sum(judge)==length(meet)-1;
    disp('Route found. The meeting time:')
    disp((meet))
else disp((meet))
    disp('Mission impossible. Please speed up or assign more chargers.')
end