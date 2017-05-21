#include <iostream>
#include <math.h>
#include <cmath>

int main() {
	int i;
	std::cout << "Wprowadź i:";
	std::cin >> i;
	std::cout << std::endl;
	float tab[i][4];
	for(int k=0; k<i; ++k) {
		std::cout << "Dla i=" << k+1 << std::endl;
		std::cout << "Podaj a(i-1):";
		std::cin >> tab[k][0];
		std::cout << std::endl;
		std::cout << "Podaj d(i)=";
		std::cin >> tab[k][1];
		std::cout << std::endl;
		std::cout << "Podaj alfa(i-1)=";
		std::cin >> tab[k][2];
		std::cout << std::endl;
		std::cout << "Podaj theta(i)=";
		std::cin >> tab[k][3];
		std::cout << std::endl;
	}
	for (int j=0; j<i; ++j) {	
		float fi = atan2(cos(tab[j][3])*sin(tab[j][2]), cos(tab[j][2]));
		float theta = atan2(-1*sin(tab[j][3])*sin(tab[j][2]), sqrt(1 - cos(tab[j][3])*sin(tab[j][2])*cos(tab[j][3])*sin(tab[j][2])));
		float psi = atan2(sin(tab[j][3])*cos(tab[j][2]), cos(tab[j][3]));
		std::cout << "Dla i=" << j+1 << ": ROLL: " << psi << " PITCH: " << theta << " YAW: " << fi << std::endl;
	}
}
