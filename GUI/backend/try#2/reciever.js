const fs = require('fs');

// File path
const filePath = 'data.txt';

// Read file asynchronously
fs.readFile(filePath, 'utf8', (err, data) => {
  if (err) {
    console.error('Error reading file:', err);
    return;
  }
  
  // Data is the contents of the file
  console.log('File contents:', data);
});
