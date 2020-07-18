s=size(FC);
count=1;
for i=1:s(2)
    ss=size(FC{i});
    temp=FC{i};
    jj=1;
    for j=count:count+ss(2)-1
        OUTPUT(1,j)=temp(1,jj);
        OUTPUT(2,j)=temp(2,jj);
        jj=jj+1;
    end
    count=count+ss(2);
end


s=size(XC);
count=1;
for i=1:s(2)
    ss=size(XC{i});
    temp=XC{i};
    jj=1;
    for j=count:count+ss(2)-1
        INPUT(1,j)=temp(1,jj)*1e7;
        INPUT(2,j)=temp(2,jj)*1e7;
        INPUT(3,j)=temp(3,jj)*1e7;
        INPUT(4,j)=temp(4,jj)*1e7;
        jj=jj+1;
    end
    count=count+ss(2);
end
% 
% net=feedforwardnet(100);
% net.trainFcn = 'trainbr'
% net.trainParam.epochs=2000;
% B1NN=train(net,INPUT,OUTPUT,'useParallel','yes','showResources','yes')
