/*

 */
#include <iostream>
#include <armadillo>

using namespace arma;
using namespace std;

int main(){
	mat A(3,2);
	mat B(3,2);
	
	A.fill(2);
	
	//A << -1 << 2 << endr
		//<< 3 << 5;
		
	B = A + 1;

	cout << B << endl;
	
	mat C(2,3);
	mat D(3,3);
	
	C << 1 << 2 << 3 << endr
	<< 4 << 5 << 6;
	
	D= A*C;
}

