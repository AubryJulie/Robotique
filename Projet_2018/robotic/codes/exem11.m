subject = iread('greenscreen.jpg', 'double');
r = subject(:,:,1);
g = subject(:,:,2);
b = subject(:,:,3);
mask = (g < r) | (g < b);
mask3 = icolor( idouble(mask) );
bg = isamesize(iread('road.png', 'double'), subject);
idisp( subject.*mask3 + bg.*(1-mask3) );


vid = Movie('LeftBag.mpg');
bg = vid.grab();
sigma = 2;
while 1
im = vid.grab;
if isempty(im), break; end
d = im-bg;
d = max(min(d, sigma), -sigma); % apply c(.)
bg=bg+d;
idisp(bg); drawnow
end


K = ones(21,21) / 21^2;
lena = iread('lena.pgm', 'double');
idisp( iconv(K, lena) );

lths = iread('lths.tif', 'double');
idisp( iconv(K, lths) );
K = kgauss(5);
idisp( iconv(K, lths) );



objects = iread('segmentation.png');
S = kcircle(3);
closed = iclose(objects, S);
clean = iopen(closed, S);

eroded = imorph(clean, kcircle(1), 'min');
idisp(clean-eroded);
skeleton = ithin(clean, 2);