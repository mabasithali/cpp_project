#include <string.h>

#include <iostream>

class String {
 private:
  char* m_Buffer;
  unsigned int m_Size;

 public:
  String(const char* string) {
    m_Size = strlen(string);
    m_Buffer = new char[m_Size + 1u];
    for (unsigned int ii = 0; ii < m_Size + 1u; ii++) {
      m_Buffer[ii] = string[ii];
    }
  }
  String(String& other) : m_Size(other.m_Size) {
    m_Buffer = new char[m_Size + 1u];
    memcpy(m_Buffer, other.m_Buffer, m_Size + 1u);
  }
  friend std::ostream& operator<<(std::ostream& stream, const String& string);
  ~String() { delete[] m_Buffer; }
};

std::ostream& operator<<(std::ostream& stream, const String& string) {
  stream << string.m_Buffer;
  return stream;
}

int main() {
  String first = "Basith";
  String copy_first = first;
  std::cout << first << std::endl;
  std::cout << copy_first << std::endl;
  return 0;
}
