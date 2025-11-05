#define SSID "your-ssid"
#define PSWD "your-password"
// use an MD5 hash generator to create the hash from your plain text password
// This is to prevent people from decrypting your OTA password from the binary
// Note however that MD5 is not considered secure against determined attacks
#define OTA_PASSWORD_HASH "your-ota-password-hash" 