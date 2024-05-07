#include "main.h"


class MyMath {
 private:
  const double kPi = 3.14156;

 public:
  double Sum_(double a, double b) {
        return a + b;
    }

  double Square_(double a) {
        return a * a;
    }
    double Subtraction_(double a, double b) {
      return a - b;
    }

    double Division_(double a, double b) {
      return a / b;
    }

    double CircleSurface_(double radius) {
      return kPi * radius * radius;
    }

    double CircleCircumference_(double radius) {
      return 2 * kPi * radius;
    }

    std::vector<double> VectorProduct_(const std::vector<double>& vec1, const std::vector<double>& vec2) {
      std::vector<double> result(3);
      result[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
      result[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
      result[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
      return result;
    }


    double DotProduct_(const std::vector<double>& vec1, const std::vector<double>& vec2) {
      double result = 0.0;
      for (int i = 0; i < vec1.size(); i++) {
        result += vec1[i] * vec2[i];
      }
      return result;
    }

    std::vector<std::vector<double>> MatrixMultiplication_(const std::vector<std::vector<double>>& matrix1, const std::vector<std::vector<double>>& matrix2) {
      std::vector<std::vector<double>> result(2, std::vector<double>(2));
      result[0][0] = matrix1[0][0] * matrix2[0][0] + matrix1[0][1] * matrix2[1][0];
      result[0][1] = matrix1[0][0] * matrix2[0][1] + matrix1[0][1] * matrix2[1][1];
      result[1][0] = matrix1[1][0] * matrix2[0][0] + matrix1[1][1] * matrix2[1][0];
      result[1][1] = matrix1[1][0] * matrix2[0][1] + matrix1[1][1] * matrix2[1][1];
      return result;
    }

    double Sin_(double a) {
      return std::sin(a);
    }

};

int main() {
  MyMath math_;
 
  int choice;
  std::cout << "Choose an operation:\n";
  std::cout << "1. Sum\n";
  std::cout << "2. Square\n";
  std::cout << "3. Subtraction\n";
  std::cout << "4. Division\n";
  std::cout << "5. Circle Surface\n";
  std::cout << "6. Circle Circumference\n";
  std::cout << "7. Vector Product\n";
  std::cout << "8. Matrix Multiplication\n";
  std::cout << "9. Sin\n";
  std::cin >> choice;

  double result;
  double num1, num2;
  std::vector<double> vec1, vec2;
  std::vector<std::vector<double>> matrix1, matrix2;
  double radius;

  switch (choice) {
    case 1:{
      std::cout << "Enter two numbers: ";
      std::cin >> num1 >> num2;
      result = math_.Sum_(num1, num2);
      std::cout << "Sum is = " << result << std::endl;
      break;
    }
    case 2:{
      std::cout << "Enter a number: ";
      std::cin >> num1;
      result = math_.Square_(num1);
      std::cout << "Square is = " << result << std::endl;
      break;
    }
    case 3:{
      std::cout << "Enter two numbers: ";
      std::cin >> num1 >> num2;
      result = math_.Subtraction_(num1, num2);
      std::cout << "Subtraction is = " << result << std::endl;
      break;
    }
    case 4:{
      std::cout << "Enter two numbers: ";
      std::cin >> num1 >> num2;
      result = math_.Division_(num1, num2);
      std::cout << "Division is = " << result << std::endl;
      break;
    }
    case 5:{
      std::cout << "Enter the radius: ";
      std::cin >> radius;
      result = math_.CircleSurface_(radius);
      std::cout << "Circle Surface is = " << result << std::endl;
      break;
    }
    case 6:{
      std::cout << "Enter the radius: ";
      std::cin >> radius;
      result = math_.CircleCircumference_(radius);
      std::cout << "Circle Circumference is = " << result << std::endl;
      break;
    }
    case 7:{
      std::cout << "Enter the first vector (3 elements): ";
      vec1.resize(3);
      std::cin >> vec1[0] >> vec1[1] >> vec1[2];
      std::cout << "Enter the second vector (3 elements): ";
      vec2.resize(3);
      std::cin >> vec2[0] >> vec2[1] >> vec2[2];
      double dotProduct = math_.DotProduct_(vec1, vec2);
      std::cout << "Dot Product is = " << dotProduct << std::endl;
      break;
    }
    case 8:{
      std::cout << "Enter the first matrix (2x2 elements):\n";
      matrix1.resize(2, std::vector<double>(2));
      std::cin >> matrix1[0][0] >> matrix1[0][1] >> matrix1[1][0] >> matrix1[1][1];
      std::cout << "Enter the second matrix (2x2 elements):\n";
      matrix2.resize(2, std::vector<double>(2));
      std::cin >> matrix2[0][0] >> matrix2[0][1] >> matrix2[1][0] >> matrix2[1][1];
      std::vector<std::vector<double>> matrixResult = math_.MatrixMultiplication_(matrix1, matrix2);
      std::cout << "Matrix Multiplication is = \n";
      std::cout << matrixResult[0][0] << " " << matrixResult[0][1] << "\n";
      std::cout << matrixResult[1][0] << " " << matrixResult[1][1] << std::endl;
      break;
    }
    case 9:{
      std::cout << "Enter a number: ";
      std::cin >> num1;
      result = math_.Sin_(num1);
      std::cout << "Sin is = " << result << std::endl;
      break;
    }
    default:
      std::cout << "Invalid choice!" << std::endl;
      break;
  }

  return 0;
}
