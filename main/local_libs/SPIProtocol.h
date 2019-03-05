#pragma once

void SPIEncrypt(unsigned char x_separated[5],int x)
{
	const unsigned int char_size = 8;
	unsigned int hash = 0;
	for (int i = 0; i < 4; i++)
	{
		x_separated[i] = ((x >> char_size * i) & 0xFF);
		hash += x_separated[i];
	}
	x_separated[4] = (unsigned char)(hash % 256);
}

bool SPIRestoreInt(const unsigned char* translated, int &restored){
	int hash = 0;
	for(int i=0; i<4; i++){
		hash += translated[i];
	}
	hash %= 256;
	if (hash == translated[4]) {
		restored = *(int*)(translated);
		return true;
	}else{
		return false;
	}
}

bool SPIRestoreUnsignedChar(const unsigned char* translated,unsigned char &restored){
	if (translated[0] == translated[1]) {
		restored = *(unsigned char*)(&translated[0]);
		return true;
	}else{
		return false;
	}
}
