import type { Metadata } from 'next';
import './globals.css';

export const metadata: Metadata = {
  title: 'SENTIO HUD',
  description: 'Real-time monitoring and control dashboard for SENTIO robot',
  viewport: {
    width: 'device-width',
    initialScale: 1,
    maximumScale: 5,
  },
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en">
      <body className="antialiased">
        {children}
      </body>
    </html>
  );
}
