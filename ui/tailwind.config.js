/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    './src/pages/**/*.{js,ts,jsx,tsx,mdx}',
    './src/components/**/*.{js,ts,jsx,tsx,mdx}',
    './src/app/**/*.{js,ts,jsx,tsx,mdx}',
  ],
  theme: {
    extend: {
      colors: {
        sentiment: {
          positive: '#10b981',
          neutral: '#6b7280',
          negative: '#ef4444',
          excited: '#f59e0b',
          calm: '#3b82f6',
        },
        sentio: {
          primary: '#0f172a',
          secondary: '#1e293b',
          accent: '#06b6d4',
          success: '#10b981',
          warning: '#f59e0b',
          error: '#ef4444',
        },
      },
      borderRadius: {
        DEFAULT: '0.5rem',
      },
      fontFamily: {
        mono: ['Fira Code', 'monospace'],
        sans: ['Inter', 'sans-serif'],
      },
    },
  },
  plugins: [],
};
