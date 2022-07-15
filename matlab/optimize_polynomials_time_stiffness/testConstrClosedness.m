



x = [0:0.1:10]
for i=1:length(x)
    v(i) = computeConstrClosedness(0.2,  x(i),5,25);
end
plot(x,v)
    

x = [5:0.1:25]
for i=1:length(x)
    v(i) = computeConstrClosedness(0.2,  x(i),5,25);
end
plot(x,v)


x = [-15:0.1:0]
for i=1:length(x)
    v(i) = computeConstrClosedness(0.2,  x(i),-15,inf);
end
plot(x,v)
