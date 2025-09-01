/**
 * Minimal Colorful Console Logger
 * 
 * Provides simple, colorful console logging functionality
 * with support for both fancy and plain logging formats
 */

const Logger = {
  COLORS: {
    reset: '\x1b[0m',
    bright: '\x1b[1m',
    dim: '\x1b[2m',
    
    // Foreground colors
    red: '\x1b[31m',
    green: '\x1b[32m',
    yellow: '\x1b[33m',
    blue: '\x1b[34m',
    magenta: '\x1b[35m',
    cyan: '\x1b[36m',
    white: '\x1b[37m',
    
    // Background colors
    bgRed: '\x1b[41m',
    bgGreen: '\x1b[42m',
    bgYellow: '\x1b[43m',
    bgBlue: '\x1b[44m'
  },
  
  LEVELS: {
    INFO: { color: 'green' },
    WARN: { color: 'yellow' },
    ERROR: { color: 'red' },
    SUCCESS: { color: 'green' },
    DEBUG: { color: 'cyan' }
  },
  
  // Configuration
  debugEnabled: false,
  fancyLoggingEnabled: false,
  
  // Enable/disable debug logs
  setDebugEnabled(enabled) {
    this.debugEnabled = enabled;
  },
  
  // Enable/disable fancy logging with colors and formatting
  setFancyLoggingEnabled(enabled) {
    this.fancyLoggingEnabled = enabled;
  },
  
  formatMessage(level, module, message) {
    // Use fancy formatting if enabled
    if (this.fancyLoggingEnabled) {
      const timestamp = new Date().toISOString().replace('T', ' ').substr(0, 19);
      const levelConfig = this.LEVELS[level] || this.LEVELS.INFO;
      const color = this.COLORS[levelConfig.color];
      const resetColor = this.COLORS.reset;
      const brightColor = this.COLORS.bright;
      
      return `${this.COLORS.dim}[${timestamp}]${resetColor} ` +
             `${color}${brightColor}${level.padEnd(7)}${resetColor} ` +
             `${this.COLORS.yellow}[${module}]${resetColor} ` +
             `${message}`;
    } else {
      // Just return the plain message for standard console.log format
      return message;
    }
  },
  
  log(level, module, message) {
    // Skip DEBUG level messages if debug logging is not enabled
    if (level === 'DEBUG' && !this.debugEnabled) {
      return;
    }
    
    console.log(this.formatMessage(level, module, message));
  },
  
  info(module, message) {
    this.log('INFO', module, message);
  },
  
  warn(module, message) {
    this.log('WARN', module, message);
  },
  
  error(module, message) {
    this.log('ERROR', module, message);
  },
  
  success(module, message) {
    this.log('SUCCESS', module, message);
  },
  
  debug(module, message) {
    this.log('DEBUG', module, message);
  },
  
  // Helper for drawing horizontal lines
  drawLine() {
    if (this.fancyLoggingEnabled) {
      console.log(`${this.COLORS.dim}${'='.repeat(80)}${this.COLORS.reset}`);
    } else {
      console.log('='.repeat(80));
    }
  },
  
  // Helper for drawing section headers
  drawHeader(title) {
    this.drawLine();
    if (this.fancyLoggingEnabled) {
      console.log(`${this.COLORS.bright}${this.COLORS.cyan}${' '.repeat(Math.floor((80 - title.length) / 2))}${title}${this.COLORS.reset}`);
    } else {
      console.log(`${' '.repeat(Math.floor((80 - title.length) / 2))}${title}`);
    }
    this.drawLine();
  }
};

module.exports = Logger;
