#include <iostream>
#include <math.h>
#include <cmath>
using namespace std;
 
int main()
{
    int i;
    cout << "ilosc przemieszczen=";
    cin >> i;
    cout << std::endl;
    double tab[i][4];
    for(int x=0;x<i;++x)
    {
        cout << "przemieszczenei nr " << x+1 << std::endl;
        cout << "(DH)a_=";
        cin >> tab[x][0];
        cout << std::endl;
        cout << "(DH)d_=";
        cin >> tab[x][1];
        cout << std::endl;
        cout << "(DH)alfa_=";
        cin >> tab[x][2];
        cout << std::endl;
        cout << "(DH)theta_=";
        cin >> tab[x][3];
        cout << std::endl;
    }
    for (int x=0; x<i; ++x)
    {
        double cos_theta = cos(tab[x][3]);
        double sin_theta = sin(tab[x][3]);
        double cos_alpha = cos(tab[x][2]);
        double sin_alpha = sin(tab[x][2]);
       
        double fi = atan2(cos_theta*sin_alpha, cos_alpha);
        double theta = atan2(-1*sin_theta*sin_alpha, sqrt(1 - cos_theta*sin_alpha*cos_theta*sin_alpha));
        double psi = atan2(sin_theta*cos_alpha, cos_theta);
       
        cout <<"**********************"<<endl << "(RPY) roll: " << psi << " pitch: " << theta << " yaw: " << fi << endl;
    }
}