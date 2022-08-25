function [x,y,id] = getXYId()
    getData = webread("http://capstone5.dothome.co.kr/getData.php");
    getData = replace(getData, '"', '');
    
    getData = split(getData(2:end-1), ',');
    
    getData = reshape(getData, height(getData)/3, []);

    x = str2num(getData{1,1});
    y = str2num(getData{1,2});
    id = str2num(getData{1,3});
end