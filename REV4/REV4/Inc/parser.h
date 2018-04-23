#ifndef parser_H_   
#define parser_H_

struct GPSObj{
    float _lat, _long, _alt;
};

struct GPSObj parser(char *data);

#endif // parser_H_