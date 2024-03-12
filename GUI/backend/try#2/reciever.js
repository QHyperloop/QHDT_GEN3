const fs = require('fs');
const pass = 'passthru.txt';
const data = 'datalog.txt';

// Author: Ryan Silverberg, Kyle ______

// TODO:
// Does not like opening too many files, need to add buffer and optimize


let previousContent = '';
function fileWatch(filepath){
  fs.watch(filepath, (filename) => { //Watch for filename
    if (filename) { //If correct filename
      fs.readFile(filepath, 'utf8', (err, data) => { //Read file
        if (err) {
          console.error('Error reading file:', err); //Throw error
          return;
        }
        if (data.trim() !== '' && data !== previousContent) { //If data is not the same as previous changes
          console.log('Updated file contents:', data);
          previousContent = data;
        }
      });
    } else {
      console.log('File not found.');
    }
  });
}


const Interval1 = setInterval(() => fileWatch(pass), 1);

const Interval2 = setInterval(() => fileWatch(data), 1);
