% findsing.m - locate a singularity of the Intelledex by trial and error

% DC Deno 9-26-91

n = 80;	% number of tries
thsing = [];
cnsing = 0;

%for i = 1:n,
%    th = rand(1,6);
%    condno = cond(jacobian(th));
%    if condno > cnsing,
%        cnsing = condno;
%        thsing = th;
%    end
%end

thsing = [];
cnsing = [];

for i = 1:n,
    th = 2*pi*rand(1,6);
    condno = cond(jacobian(th));
    cnsing = [cnsing; condno];
    thsing = [thsing; th];
end

[csort,isort] = sort(cnsing);
isort = flipud(isort);

cnsing(isort(1:20))
pause
thsing(isort(1:20),:) 
pause

%disp('The most singular configuration encountered was:');
%thsing
%disp('and had condition number');
%cnsing
